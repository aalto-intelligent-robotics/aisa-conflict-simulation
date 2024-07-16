#!/usr/bin/env python
import os
import time
from pathlib import Path
import random
import rospy
import rosgraph
import torch
import numpy as np
import cv2
from PIL import Image, ImageDraw
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from unity_robotics_demo_msgs.msg import AckermannDrive, Trajectory, PosRot


NODE_NAME_SUBSCRIBE = "lane_following"
NODE_NAME_PUBLISH = "ackermanncontrol_publisher"
TOPIC_NAME_READ_IMAGES = "wasp_cam/compressed"
TOPIC_NAME_LANES_OVERLAY_IMAGE = "detected_lanes_image"
TOPIC_NAME_PUBLISH_CONTROL = "ackermanncontrol"
TOPIC_NAME_TRAJECTORY = "trajectory"

#--------------------------------------------------
# UPDATE THESE:

# Path to the lane detection model, path in Docker
MODEL_PATH = "/home/aisa/code/src/unity_robotics_demo/scripts/models/fastai_model.pth"

# Where to save the visualization from the lane detection on the Unity camera image, path in Docker
VIZ_SAVE_PATH = "/home/aisa/code/src/unity_robotics_demo/scripts/viz.png"
#--------------------------------------------------


# =========================
# camera_geometry.py
# =========================
def get_intrinsic_matrix(field_of_view_deg, image_width, image_height):
    # For our Carla camera alpha_u = alpha_v = alpha
    # alpha can be computed given the cameras field of view via
    field_of_view_rad = field_of_view_deg * np.pi / 180
    alpha = (image_width / 2.0) / np.tan(field_of_view_rad / 2.0)
    Cu = image_width / 2.0
    Cv = image_height / 2.0
    return np.array([[alpha, 0, Cu], [0, alpha, Cv], [0, 0, 1.0]])


def project_polyline(polyline_world, trafo_world_to_cam, K):
    x, y, z = polyline_world[:, 0], polyline_world[:, 1], polyline_world[:, 2]
    homvec = np.stack((x, y, z, np.ones_like(x)))
    proj_mat = K @ trafo_world_to_cam[:3, :]
    pl_uv_cam = (proj_mat @ homvec).T
    u = pl_uv_cam[:, 0] / pl_uv_cam[:, 2]
    v = pl_uv_cam[:, 1] / pl_uv_cam[:, 2]
    return np.stack((u, v)).T


class CameraGeometry:
    def __init__(
        self,
        height=1.3,
        yaw_deg=0,
        pitch_deg=-5,
        roll_deg=0,
        image_width=1024,
        image_height=512,
        field_of_view_deg=45,
    ):
        # scalar constants
        self.height = height
        self.pitch_deg = pitch_deg
        self.roll_deg = roll_deg
        self.yaw_deg = yaw_deg
        self.image_width = image_width
        self.image_height = image_height
        self.field_of_view_deg = field_of_view_deg
        # camera intriniscs and extrinsics
        self.intrinsic_matrix = get_intrinsic_matrix(
            field_of_view_deg, image_width, image_height
        )
        self.inverse_intrinsic_matrix = np.linalg.inv(self.intrinsic_matrix)
        ## Note that "rotation_cam_to_road" has the math symbol R_{rc} in the book
        yaw = np.deg2rad(yaw_deg)
        pitch = np.deg2rad(pitch_deg)
        roll = np.deg2rad(roll_deg)
        cy, sy = np.cos(yaw), np.sin(yaw)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cr, sr = np.cos(roll), np.sin(roll)
        rotation_road_to_cam = np.array(
            [
                [cr * cy + sp * sr + sy, cr * sp * sy - cy * sr, -cp * sy],
                [cp * sr, cp * cr, sp],
                [cr * sy - cy * sp * sr, -cr * cy * sp - sr * sy, cp * cy],
            ]
        )
        self.rotation_cam_to_road = (
            rotation_road_to_cam.T
        )  # for rotation matrices, taking the transpose is the same as inversion
        self.translation_cam_to_road = np.array([0, -self.height, 0])
        self.trafo_cam_to_road = np.eye(4)
        self.trafo_cam_to_road[0:3, 0:3] = self.rotation_cam_to_road
        self.trafo_cam_to_road[0:3, 3] = self.translation_cam_to_road
        # compute vector nc. Note that R_{rc}^T = R_{cr}
        self.road_normal_camframe = self.rotation_cam_to_road.T @ np.array([0, 1, 0])

    def camframe_to_roadframe(self, vec_in_cam_frame):
        return (
            self.rotation_cam_to_road @ vec_in_cam_frame + self.translation_cam_to_road
        )

    def uv_to_roadXYZ_camframe(self, u, v):
        # NOTE: The results depend very much on the pitch angle (0.5 degree error yields bad result)
        # Here is a paper on vehicle pitch estimation:
        # https://refubium.fu-berlin.de/handle/fub188/26792
        uv_hom = np.array([u, v, 1])
        Kinv_uv_hom = self.inverse_intrinsic_matrix @ uv_hom
        denominator = self.road_normal_camframe.dot(Kinv_uv_hom)
        return self.height * Kinv_uv_hom / denominator

    def uv_to_roadXYZ_roadframe(self, u, v):
        r_camframe = self.uv_to_roadXYZ_camframe(u, v)
        return self.camframe_to_roadframe(r_camframe)

    def uv_to_roadXYZ_roadframe_iso8855(self, u, v):
        X, Y, Z = self.uv_to_roadXYZ_roadframe(u, v)
        return np.array(
            [Z, -X, -Y]
        )  # read book section on coordinate systems to understand this

    def precompute_grid(self, dist=60):
        cut_v = int(self.compute_minimum_v(dist=dist) + 1)
        xy = []
        for v in range(cut_v, self.image_height):
            for u in range(self.image_width):
                X, Y, Z = self.uv_to_roadXYZ_roadframe_iso8855(u, v)
                xy.append(np.array([X, Y]))
        xy = np.array(xy)
        return cut_v, xy

    def compute_minimum_v(self, dist):
        """
        Find cut_v such that pixels with v<cut_v are irrelevant for polynomial fitting.
        Everything that is further than `dist` along the road is considered irrelevant.
        """
        trafo_road_to_cam = np.linalg.inv(self.trafo_cam_to_road)
        point_far_away_on_road = trafo_road_to_cam @ np.array([0, 0, dist, 1])
        uv_vec = self.intrinsic_matrix @ point_far_away_on_road[:3]
        uv_vec /= uv_vec[2]
        cut_v = uv_vec[1]
        return cut_v


# =========================
# lane_detector.py
# =========================
class LaneDetector:
    def __init__(self, cam_geom=CameraGeometry(), model_path=MODEL_PATH):
        self.cg = cam_geom
        self.cut_v, self.grid = self.cg.precompute_grid()
        if torch.cuda.is_available():
            self.device = "cuda"
            self.model = torch.load(model_path).to(self.device)
        else:
            self.model = torch.load(model_path, map_location=torch.device("cpu"))
            self.device = "cpu"
        self.model.eval()
        self.output = None
        self.model_output = None

    def read_imagefile_to_array(self, filename):
        image = cv2.imread(filename)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        return image

    def detect_from_file(self, filename):
        img_array = self.read_imagefile_to_array(filename)
        return self.detect(img_array)

    def _predict(self, img):
        with torch.no_grad():
            image_tensor = img.transpose(2, 0, 1).astype("float32") / 255
            x_tensor = torch.from_numpy(image_tensor).to(self.device).unsqueeze(0)
            output = self.model.forward(x_tensor)
            self.output = output.cpu().numpy()
            # print('output', output.shape)
            # plt.imshow(output[0][0].cpu())
            # plt.show()
            model_output = torch.softmax(output, dim=1).cpu().numpy()
            # print('model_output', model_output.shape)

            # OLD: model_output = torch.softmax(self.model.forward(x_tensor), dim=1).cpu().numpy()

            self.model_output = model_output
        return model_output

    def detect(self, img_array):
        model_output = self._predict(img_array)
        background, left, right = (
            model_output[0, 0, :, :],
            model_output[0, 1, :, :],
            model_output[0, 2, :, :],
        )
        return background, left, right

    def fit_poly(self, probs):
        probs_flat = np.ravel(probs[self.cut_v :, :])
        mask = probs_flat > 0.1
        if mask.sum() > 0:
            coeffs = np.polyfit(
                self.grid[:, 0][mask], self.grid[:, 1][mask], deg=3, w=probs_flat[mask]
            )
        else:
            coeffs = np.array([0.0, 0.0, 0.0, 0.0])
        return np.poly1d(coeffs)

    def __call__(self, image):
        if isinstance(image, str):
            image = self.read_imagefile_to_array(image)
        left_poly, right_poly, _, _ = self.get_fit_and_probs(image)
        return left_poly, right_poly

    def get_fit_and_probs(self, img):
        _, left, right = self.detect(img)
        left_poly = self.fit_poly(left)
        right_poly = self.fit_poly(right)
        return left_poly, right_poly, left, right


# =========================
# lane_detection.py
# =========================


def get_trajectory_from_lane_detector(ld, image):
    # get lane boundaries using the lane detector
    image = cv2.resize(image, (1024, 512), interpolation=cv2.INTER_AREA)

    print("Image for lane detection:", image.shape, image.dtype)

    poly_left, poly_right, left_mask, right_mask = ld.get_fit_and_probs(image)
    # trajectory to follow is the mean of left and right lane boundary
    # note that we multiply with -0.5 instead of 0.5 in the formula for y below
    # according to our lane detector x is forward and y is left, but
    # according to Carla x is forward and y is right.

    # In Unity: x is right, y is up, z is forward: maybe no adjustment needed

    print(f"poly_left: {poly_left}")
    print(f"poly_right: {poly_right}")

    # x = np.arange(-2, 60, 1.0)
    # x = np.arange(10, 60, 1.0)
    x = np.arange(2, 20, 1.0)
    if poly_left == 0:
        poly_left = poly_right
    elif poly_right == 0:
        poly_right = poly_left
    y = 0.5 * (poly_left(x) + poly_right(x))

    print(f"x: {x}")
    print(f"y: {y}")
    # x,y is now in coordinates centered at camera, but camera is 0.5 in front of vehicle center
    # hence correct x coordinates
    # x += 1.0
    # x -= 1.0
    # x += 0.5
    # y += 0.1
    traj = np.stack((x, y)).T
    # print(f"traj: \n{traj}")
    return (
        traj,
        ld_detection_overlay(image, left_mask, right_mask, False),
        ld_detection_overlay(image, left_mask, right_mask, True),
        left_mask,
        right_mask,
    )


def request_takeover(left_mask: np.ndarray, right_mask: np.ndarray) -> bool:
    """ Checks if takeover should be requested given the detected lanes data
    Args:
        left_mask, right_mask: Masks of detected lanes, containing 
            probabilities of each pixel elonding to the left or right lane
    Returns:
        Whether takeover should be requested given the detected lanes
    """
    print(f"Left mask: {left_mask.min():.4f}-{left_mask.max():.4f}")
    print(f"Right mask: {right_mask.min():.4f}-{right_mask.max():.4f}")

    left_mask[left_mask < 0.1e-10] = 0
    right_mask[right_mask < 0.1e-10] = 0
    
    detection_threshold = 0.4
    takeover_required = (
        left_mask.max() < detection_threshold and right_mask.max() < detection_threshold
    )

    return takeover_required


def detect_lanes(image):
    """Detect lanes in given image and returns trajectory, the lanes image and need for takeover

    Args:
        image: the image to detect lanes in

    Returns:
        traj: Trajectory based on deteced lanes
        viz: The image with dtected lanes
        takeover_required: Based on the model confidence on lane detection, is takeover required
    """
    model_path = MODEL_PATH
    ld = get_lane_detector(model_path)

    traj, viz, viz_overlay, left_mask, right_mask = get_trajectory_from_lane_detector(ld, image)
    takeover_required = request_takeover(left_mask, right_mask)
    
    im = Image.fromarray(viz)
    # im.draw.text((0, 0),"Sample Text",(255,255,255))
    draw = ImageDraw.Draw(im)
    draw.text((0, 0), f"Left mask: {left_mask.min():.4f}-{left_mask.max():.4f}",(255,255,255))
    draw.text((0, 10), f"Right mask: {right_mask.min():.4f}-{right_mask.max():.4f}",(255,255,255))

    im.save(VIZ_SAVE_PATH)

    return traj, viz, viz_overlay, takeover_required


def ld_detection_overlay(image, left_mask, right_mask, overlay=True):
    if overlay:
        res = np.zeros_like(image)
    else:
        res = image #.putalpha(0.5)
    # res = np.zeros_like(image)
    # res[left_mask > 0.5, :] = [0, 0, 255]
    # res[right_mask > 0.5, :] = [255, 0, 0]

    res[left_mask > 0.4, :] = [255, 0, 0]
    res[right_mask > 0.4, :] = [255, 0, 0]

    res[left_mask > 0.5, :] = [255, 160, 0]
    res[right_mask > 0.5, :] = [255, 160, 0]

    res[left_mask > 0.8, :] = [255, 255, 0]
    res[right_mask > 0.8, :] = [255, 255, 0]

    res[left_mask > 0.9, :] = [0, 255, 0]
    res[right_mask > 0.9, :] = [0, 255, 0]

    # res[left_mask > 0.8 and left_mask < 0.9, :] = [255, 255, 0]
    # res[right_mask > 0.8 and right_mask < 0.9, :] = [255, 255, 0]

    return res


def get_lane_detector(model_path):
    cg = CameraGeometry()
    ld = LaneDetector(model_path=Path(model_path).absolute(), cam_geom=cg)
    return ld


# ==============================
# get_target_point.py
# ==============================


# Function from https://stackoverflow.com/a/59582674/2609987
def circle_line_segment_intersection(
    circle_center, circle_radius, pt1, pt2, full_line=True, tangent_tol=1e-9
):
    """Find the points at which a circle intersects a line-segment.  This can happen at 0, 1, or 2 points.

    :param circle_center: The (x, y) location of the circle center
    :param circle_radius: The radius of the circle
    :param pt1: The (x, y) location of the first point of the segment
    :param pt2: The (x, y) location of the second point of the segment
    :param full_line: True to find intersections along full line - not just in the segment.  False will just return intersections within the segment.
    :param tangent_tol: Numerical tolerance at which we decide the intersections are close enough to consider it a tangent
    :return Sequence[Tuple[float, float]]: A list of length 0, 1, or 2, where each element is a point at which the circle intercepts a line segment.

    Note: We follow: http://mathworld.wolfram.com/Circle-LineIntersection.html
    """

    (p1x, p1y), (p2x, p2y), (cx, cy) = pt1, pt2, circle_center
    (x1, y1), (x2, y2) = (p1x - cx, p1y - cy), (p2x - cx, p2y - cy)
    dx, dy = (x2 - x1), (y2 - y1)
    dr = (dx**2 + dy**2) ** 0.5
    big_d = x1 * y2 - x2 * y1
    discriminant = circle_radius**2 * dr**2 - big_d**2

    if discriminant < 0:  # No intersection between circle and line
        return []
    else:  # There may be 0, 1, or 2 intersections with the segment
        intersections = [
            (
                cx
                + (big_d * dy + sign * (-1 if dy < 0 else 1) * dx * discriminant**0.5)
                / dr**2,
                cy + (-big_d * dx + sign * abs(dy) * discriminant**0.5) / dr**2,
            )
            for sign in ((1, -1) if dy < 0 else (-1, 1))
        ]  # This makes sure the order along the segment is correct
        if (
            not full_line
        ):  # If only considering the segment, filter out intersections that do not fall within the segment
            fraction_along_segment = [
                (xi - p1x) / dx if abs(dx) > abs(dy) else (yi - p1y) / dy
                for xi, yi in intersections
            ]
            intersections = [
                pt
                for pt, frac in zip(intersections, fraction_along_segment)
                if 0 <= frac <= 1
            ]
        if (
            len(intersections) == 2 and abs(discriminant) <= tangent_tol
        ):  # If line is tangent to circle, return just one point (as both intersections have same location)
            return [intersections[0]]
        else:
            return intersections


def get_target_point(lookahead, polyline):
    """Determines the target point for the pure pursuit controller

    Parameters
    ----------
    lookahead : float
        The target point is on a circle of radius `lookahead`
        The circle's center is (0,0)
    poyline: array_like, shape (M,2)
        A list of 2d points that defines a polyline.

    Returns:
    --------
    target_point: numpy array, shape (,2)
        Point with positive x-coordinate where the circle of radius `lookahead`
        and the polyline intersect.
        Return None if there is no such point.
        If there are multiple such points, return the one that the polyline
        visits first.
    """
    intersections = []
    for j in range(len(polyline) - 1):
        pt1 = polyline[j]
        pt2 = polyline[j + 1]
        intersections += circle_line_segment_intersection(
            (0, 0), lookahead, pt1, pt2, full_line=False
        )
    filtered = [p for p in intersections if p[0] > 0]
    if len(filtered) == 0:
        return None
    return filtered[0]


# =============================
# pure_pursuit.py
# =============================

# TODO: Tune parameters of PID with these global variables
param_Kp = 2
param_Ki = 0
param_Kd = 0
# TODO: Tune parameters of Pure Pursuit with these global variables
param_K_dd = 0.4


class PurePursuit:
    def __init__(self, K_dd=param_K_dd, wheel_base=2.65, waypoint_shift=1.4):
        self.K_dd = K_dd
        self.wheel_base = wheel_base
        self.waypoint_shift = waypoint_shift

    def get_control(self, waypoints, speed):
        # transform x coordinates of waypoints such that coordinate origin is in rear wheel
        waypoints[:, 0] += self.waypoint_shift
        look_ahead_distance = np.clip(self.K_dd * speed, 3, 20)

        track_point = get_target_point(look_ahead_distance, waypoints)
        if track_point is None:
            return 0

        alpha = np.arctan2(track_point[1], track_point[0])

        # Change the steer output with the lateral controller.
        steer = np.arctan((2 * self.wheel_base * np.sin(alpha)) / look_ahead_distance)

        # undo transform to waypoints
        waypoints[:, 0] -= self.waypoint_shift
        return steer


class PIDController:
    def __init__(self, Kp, Ki, Kd, set_point):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.set_point = set_point
        self.int_term = 0
        self.derivative_term = 0
        self.last_error = None

    def get_control(self, measurement, dt):
        error = self.set_point - measurement
        self.int_term += error * self.Ki * dt
        if self.last_error is not None:
            self.derivative_term = (error - self.last_error) / dt * self.Kd
        self.last_error = error
        return self.Kp * error + self.int_term + self.derivative_term


class PurePursuitPlusPID:
    def __init__(
        self,
        pure_pursuit=PurePursuit(),
        pid=PIDController(param_Kp, param_Ki, param_Kd, 0),
    ):
        self.pure_pursuit = pure_pursuit
        self.pid = pid

    def get_control(self, waypoints, speed, desired_speed, dt):
        self.pid.set_point = desired_speed
        a = self.pid.get_control(speed, dt)
        steer = self.pure_pursuit.get_control(waypoints, speed)
        return a, steer


# =============================
# control from lane detection
# =============================


def image_message_to_cv_image(image_message: CompressedImage) -> cv2.Mat:
    """Converts the image from the message to OpenCV format.
        http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

        Args:
            image_message: ROS image message
    s
        Returns:
            The image from the message, converted to OpenCV format.
    """
    bridge: CvBridge = CvBridge()
    cv_image = bridge.compressed_imgmsg_to_cv2(
        image_message, desired_encoding="passthrough"
    )

    return cv_image


def cv_image_to_image_message(image) -> CompressedImage:
    """Converts from image to CompressedImage.
        http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

        Args:
            image: cv2 image
    s
        Returns:
            CompressedImage
    """
    bridge: CvBridge = CvBridge()
    compressed_image = bridge.cv2_to_compressed_imgmsg(image)

    return compressed_image


def publish_control_message(
    steering_angle: float,
    steering_angle_velocity: float,
    speed: float,
    acceleration: float,
    jerk: float,
    publish_trajectory:bool=True
) -> None:
    """Publish the Ackermann control message and trajectory.
    Args:
    Input parameters for the Ackermann message:
        steering_angle: float
        steering_angle_velocity: float
        speed: float
        acceleration: float
        jerk: float
    """
    print("Publishing control message...")
    rospy.init_node(NODE_NAME_PUBLISH, anonymous=True)
    pub = rospy.Publisher(TOPIC_NAME_PUBLISH_CONTROL, AckermannDrive, queue_size=10)
    ackcontrol = AckermannDrive(
        steering_angle, steering_angle_velocity, speed, acceleration, jerk
    )
    # wait_for_connections(pub, TOPIC_NAME_PUBLISH_CONTROL)
    pub.publish(ackcontrol)
    print(f"Published control {ackcontrol}")

    if publish_trajectory:
        pub = rospy.Publisher(TOPIC_NAME_TRAJECTORY, Trajectory, queue_size=10)
        trajectory = Trajectory(
            
        )
        wait_for_connections(pub, TOPIC_NAME_TRAJECTORY)
        pub.publish(trajectory)
        print(f"Published trajectory {trajectory}")


def publish_trajectory(
    traj: np.ndarray
) -> None:
    """Publish predicted trajectory.
    """
    print("Pubishing predicted trajectory message...")
    pub = rospy.Publisher(TOPIC_NAME_TRAJECTORY, Trajectory, queue_size=10)

    waypoints = []
    for waypoint in traj:
        pos_rot = PosRot(pos_x=waypoint[0], pos_y=waypoint[1])
        waypoints.append(pos_rot)

    trajectory = Trajectory(waypoints)
    wait_for_connections(pub, TOPIC_NAME_TRAJECTORY)
    pub.publish(trajectory)
    print(f"Published trajectory {trajectory}")


def publish_lane_image(image: CompressedImage):
    """Publishing the lanes overlay message.
    """
    print("Pubishing lane image message...")
    topic_name = TOPIC_NAME_LANES_OVERLAY_IMAGE
    pub = rospy.Publisher(topic_name, CompressedImage, queue_size=10)
    wait_for_connections(pub, topic_name)
    pub.publish(image)


def wait_for_connections(pub: rospy.Publisher, topic: str) -> None:
    """Try to connect to ROS topic.
    Args:
        pub: ROS topic publisher
        topic: Name of the ROS topic
    """
    ros_master = rosgraph.Master("/rostopic")
    topic = rosgraph.names.script_resolve_name("rostopic", topic)
    num_subs = 0
    for sub in ros_master.getSystemState()[1]:
        if sub[0] == topic:
            num_subs += 1

    for i in range(10):
        if pub.get_num_connections() == num_subs:
            return
        time.sleep(0.1)
    raise RuntimeError("Failed to get publisher.")


def process_image_and_publish_control(image_message: CompressedImage) -> None:
    """Process the posted camera image and publish control to the car depending on the result.

    Args:
    image_message: The image message from the camera topic
    """
    print("In callback process_image_and_publish_control()")
    steering_angle: float = 0
    steering_angle_velocity: float = 0
    speed: float = 0
    acceleration: float = 0
    jerk: float = 0

    # Convert the image message to OpenCV format
    image = image_message_to_cv_image(image_message)

    print(f"Converted image {image.shape}")

    # Get detected lanes
    traj, lanes, viz_overlay, takeover_required = detect_lanes(image)

    SPEED = 3
    # PID controller
    controller = PurePursuitPlusPID()
    throttle, steer = controller.get_control(
        traj, speed=SPEED, desired_speed=SPEED, dt=0.01
    )

    print(f"Speed: {SPEED}")
    print(f"Traj: {traj[0]}")
    print(f"Lanes detected {takeover_required}")

    if not takeover_required:
        # If no need to pass control, continue straight
        speed = SPEED
        acceleration = 0.02
        steering_angle = 0.1 * steer
        jerk = throttle

    publish_control_message(
        steering_angle,
        steering_angle_velocity,
        speed,
        acceleration,
        jerk,
    )

    publish_trajectory(traj)

    lane_image_message = cv_image_to_image_message(viz_overlay)
    publish_lane_image(lane_image_message)


def subscribe_for_camera_images():
    """Subscribe to the ROS topic posting camera images."""
    print(f"Subscribing for {TOPIC_NAME_READ_IMAGES}...")
    rospy.init_node(NODE_NAME_PUBLISH, anonymous=True)
    rospy.Subscriber(
        TOPIC_NAME_READ_IMAGES, CompressedImage, process_image_and_publish_control
    )
    rospy.spin()


if __name__ == "__main__":
    try:
        print("entered main()")
        subscribe_for_camera_images()
    except rospy.ROSInterruptException:
        pass
