using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using Unity.Robotics.ROSTCPConnector;

using ROS_String = RosMessageTypes.Std.StringMsg;
using ROS_PointCloud2 = RosMessageTypes.Sensor.PointCloud2Msg;
using ROS_PointCloud = RosMessageTypes.Sensor.PointCloudMsg;
using ROS_Point = RosMessageTypes.Geometry.Point32Msg;
using ROS_Channel = RosMessageTypes.Sensor.ChannelFloat32Msg;
using ROS_PointField = RosMessageTypes.Sensor.PointFieldMsg;
using System;

public class LiDAR_Reiti : MonoBehaviour
{
    public float Timescale = 1;
    public Transform emitter, SensorRotator;
    public float Range = 10, StartVerticalAngle = 2, VertScanAngRange = 26.8f, HorScanAngRange = 360, hz = 10, HorRes = 0.08f;
    int NoOfScansPerFrame = 0;
    public int lasercount = 64;
    public bool DebugDraw = true, DebugDrawDots = false;
    public float DrawTime = 0.1f;
    // float[] datacolumn;
    public bool InterpolateLocation = true;
    // StringMsg rosMsg = new StringMsg("");
    public bool send_as_pointcloud1 = true;

    public bool random_error = false;

    private ROSConnection connection;
    private Vector3 shootLaserDir = Vector3.zero;
    private float pitchAngle = 0;
    private Rigidbody attachedRB;
    private Vector3 intLoc, ScannerLoc, prevScannerLoc, velocity = Vector3.zero;
    private Transform my_transform;
    private string publish_topic = "unity/lidar";
    private List<Vector3> PointsTemp, Points;
    private ROS_PointField[] pointcloud_layout;

    // Start is called before the first frame update
    void Start()
    {
        connection = ROSConnection.GetOrCreateInstance();
        if(send_as_pointcloud1)
            connection.RegisterPublisher<ROS_PointCloud>(publish_topic);
        else
            connection.RegisterPublisher<ROS_PointCloud2>(publish_topic);
        connection.RegisterPublisher<ROS_String>("tmp");

        pointcloud_layout = new ROS_PointField[4];
        pointcloud_layout[0] = new ROS_PointField("x", 0, 7, 1);
        pointcloud_layout[1] = new ROS_PointField("y", 4 * 1, 7, 1);
        pointcloud_layout[2] = new ROS_PointField("z", 4 * 2, 7, 1);
        pointcloud_layout[2] = new ROS_PointField("intensity", 4 * 3, 7, 1);

        my_transform = transform;
        ScannerLoc = my_transform.position;
        prevScannerLoc = ScannerLoc;
        if (!SensorRotator) SensorRotator = my_transform.Find("Laser Sensor").transform;
        if (!emitter) SensorRotator = my_transform.Find("Emitter").transform;

        attachedRB = GetComponentInParent<Rigidbody>();
        PointsTemp = new List<Vector3>();
        Points = new List<Vector3>();
        // writer = new StreamWriter(Application.dataPath+"/Data.csv");
        NoOfScansPerFrame = (int)((HorScanAngRange / HorRes) * hz * Time.fixedDeltaTime);
        currentangle = -HorScanAngRange / 2;

        // writer.WriteLine("This is a data file");
        // datacolumn=new float[lasercount];
    }
    float currentangle;
    float[] ranges;
    // private StreamWriter writer;
    string rosData = "";
    short dataOffset = 14;
    // Update is called once per frame
    

    void scan()
    {
        ScannerLoc = my_transform.position;
        //velocity = (ScannerLoc - prevScannerLoc) / Time.fixedDeltaTime;
        velocity = (ScannerLoc - prevScannerLoc) / Time.deltaTime;
        //message floats format: [number of rays,number of columns, sensor Angle, min Vert Angle, vertRes,HorRes,x,y,z,rx,ry,rz[ranges]] in ros coordinates x forward y left
        if (currentangle > HorScanAngRange / 2)
        {
            currentangle = -HorScanAngRange / 2; 
            SensorRotator.localEulerAngles = new Vector3(0, -HorScanAngRange / 2, 0);

        } //completed horizontal scan

        Points.Clear();

        for (int i = 0; i < NoOfScansPerFrame; i++){
            RaycastHit hit;            

            emitter.localEulerAngles = new Vector3(-StartVerticalAngle, 0, 0);

            for (int j = 0; j < lasercount; j++){
                // Pitch
                pitchAngle = (StartVerticalAngle + j * VertScanAngRange / (lasercount - 1));
                //Debug.Log(pitchAngle);
                emitter.localEulerAngles = new Vector3(pitchAngle, currentangle, 0);                
                shootLaserDir = (emitter.forward);
                if (Physics.Raycast(ScannerLoc, shootLaserDir, out hit, Range))
                {                
                    Vector3 p = hit.point;
                    //Vector3 error = new Vector3(0,0,UnityEngine.Random.value);
                    if (DebugDraw) Debug.DrawLine(p, SensorRotator.position, Color.red, DrawTime, true);
                    else if (DebugDrawDots) Debug.DrawLine(p, p + 0.1f * Vector3.up, Color.red, DrawTime, true);
                    Vector3 global_pos = ScannerLoc + hit.point;
                    if(random_error){
                        Points.Add(new Vector3(global_pos.x,global_pos.z + (UnityEngine.Random.value/5),global_pos.y)); 
                    } else {
                        Points.Add(new Vector3(global_pos.x,global_pos.z,global_pos.y));
                    }
                    
                }
            }

            currentangle += HorRes;
            SensorRotator.localEulerAngles = new Vector3(0, currentangle, 0);
        }

        /*

        for (int i = 0; i < NoOfScansPerFrame; i++)
        { // multiple horizontal scans in 1 physics step in order to achieve the full range in the desired rate
            if (InterpolateLocation)
                ScannerLoc = Vector3.Lerp(my_transform.position, intLoc, (float)i / NoOfScansPerFrame);
            if (currentangle > HorScanAngRange / 2) { currentangle = -HorScanAngRange / 2; SensorRotator.localEulerAngles = new Vector3(0, -HorScanAngRange / 2, 0); }//rotate horizontally
            //emitter.localEulerAngles = new Vector3(-StartVerticalAngle, 0, 0);
            emitter.localEulerAngles = new Vector3(-StartVerticalAngle, currentangle, 0);
            for (int j = 0; j < lasercount; j++) //the lazer column
            {
                pitchAngle = (StartVerticalAngle + j * VertScanAngRange / (lasercount - 1));
                //emitter.localEulerAngles = new Vector3(pitchAngle, 0, 0);
                emitter.localEulerAngles = new Vector3(pitchAngle, currentangle, 0);
                shootLaserDir = (emitter.forward);
                RaycastHit hit;
                if (Physics.Raycast(ScannerLoc, shootLaserDir, out hit, Range))
                {
                    Vector3 p = hit.point;
                    if (DebugDraw) Debug.DrawLine(p, SensorRotator.position, Color.red, DrawTime, true);
                    else if (DebugDrawDots) Debug.DrawLine(p, p + 0.1f * Vector3.up, Color.red, DrawTime, true);

                    Vector3 global_pos = ScannerLoc + hit.point;

                    Points.Add(ScannerLoc + hit.point);

                }
                // datacolumn[j]=hit.distance;
            }

            // WriteData(datacolumn);
            currentangle += HorRes;
            SensorRotator.localEulerAngles = new Vector3(0, currentangle, 0);

        }
        */

        if (send_as_pointcloud1)
        {
            var pointcloud_1_points_channels = convert_to_ros_pc(Points, SensorRotator.position);

            ROS_PointCloud msg = new ROS_PointCloud(
                new RosMessageTypes.Std.HeaderMsg( //header
                    (uint)Time.frameCount,
                    new RosMessageTypes.BuiltinInterfaces.TimeMsg(
                        (uint)Math.Floor(Time.realtimeSinceStartup),
                        (uint)((Time.realtimeSinceStartup - Mathf.Floor(Time.realtimeSinceStartup)) * 1000000000)),
                    "map"),
                pointcloud_1_points_channels.Item1,
                pointcloud_1_points_channels.Item2);

            connection.Publish(publish_topic, msg);
        }
        else
        {
            byte[] point_data = convert_to_ros_pc2_points(Points);

            //TODO Error: https://forum.unity.com/threads/nullreferenceexception-on-rostcpconnector-rostopicstate-deserialize.1188382/
            // need to rebuild packages and unsubscribe from 'listening to tf messages' -> add script in new project!
            ROS_PointCloud2 ros_msg = new ROS_PointCloud2(
                new RosMessageTypes.Std.HeaderMsg( //header
                    (uint)Time.frameCount,
                    new RosMessageTypes.BuiltinInterfaces.TimeMsg(
                        (uint)Math.Floor(Time.realtimeSinceStartup),
                        (uint)((Time.realtimeSinceStartup - Mathf.Floor(Time.realtimeSinceStartup)) * 1000000000)),
                    "map"),
                (uint)1, // height (is 1 for unordered Pointcloud)
                (uint)Points.Count, // width (is length for unordered Pointcloud)
                pointcloud_layout,
                !BitConverter.IsLittleEndian,
                (uint)(4 * sizeof(float)), // point step
                (uint)(Points.Count * 4 * sizeof(float)), // row step
                point_data, // data
                true);// is dense

            Debug.LogError("Not implemented yet");
            // TODO Error is thrown here!! Check out what happens there
            ros_msg.Serialize();

            // https://docs.ros.org/en/lunar/api/sensor_msgs/html/msg/PointCloud1.html
            // https://docs.ros.org/en/lunar/api/sensor_msgs/html/msg/PointCloud2.html
            connection.Publish(publish_topic, ros_msg);
            //connection.Publish("tmp", new ROS_String("Hey, is this working?"));
        }

        prevScannerLoc = SensorRotator.position;
    }


    Vector3 UnityToRosPositionAxisConversion(Vector3 unity_axis)
    {
        return new Vector3(unity_axis.x, unity_axis.z, unity_axis.y);
    }

    Quaternion UnityToRosRotationAxisConversion(Quaternion quaternion)
    {
        return new Quaternion(quaternion.x, quaternion.z, quaternion.y, quaternion.w);
    }

    private void convert_unity_points_to_ros(List<Vector3> points, out ROS_Point[] ros_points, out ROS_Channel[] channels)
    {
        ros_points = new ROS_Point[points.Count];
        channels = new ROS_Channel[points.Count];
        float[] dist = { 1.0f };
        for(int i = 0; i < points.Count; i++)
        {
            ros_points[i] = new ROS_Point(points[i].x, points[i].z, points[i].y);
            dist[0] = Vector3.Distance(points[i], prevScannerLoc);
            channels[i] = new ROS_Channel("distance", dist);
        }
    }

    private byte[] convert_to_ros_pc2_points(List<Vector3> points)
    {
        int size_point = sizeof(float) * 4;
        byte[] output = new byte[points.Count * size_point];
        for (int i = 0; i < points.Count; i++)
        {
            Buffer.BlockCopy(BitConverter.GetBytes(points[i].x), 0, output, i * size_point, sizeof(float));
            Buffer.BlockCopy(BitConverter.GetBytes(points[i].y), 0, output, i * size_point + sizeof(float), sizeof(float));
            Buffer.BlockCopy(BitConverter.GetBytes(points[i].z), 0, output, i * size_point + sizeof(float) * 2, sizeof(float));
            Buffer.BlockCopy(BitConverter.GetBytes(Vector3.Distance(points[i], ScannerLoc)), 0, output, i * size_point + sizeof(float) * 3, sizeof(float));
        }

        return output;
    }

    private Tuple<ROS_Point[], ROS_Channel[]> convert_to_ros_pc(List<Vector3> points, Vector3 lidar_position)
    {
        ROS_Point[] ros_points = new ROS_Point[points.Count];
        ROS_Channel[] ros_channels = new ROS_Channel[1];
        float[] distances = new float[points.Count];
        for (int i = 0; i < points.Count; i++)
        {
            Vector3 current = points[i];
            ros_points[i] = new ROS_Point(current.x, current.y, current.z);
            distances[i] = Vector3.Distance(lidar_position, current);
        }
        ros_channels[0] = new ROS_Channel("distance", distances);

        return new Tuple<ROS_Point[], ROS_Channel[]>(ros_points, ros_channels);
    }

    private void Update()
    {
        //transform.position = transform.position + new Vector3(0, 0, 0.01f * (float)Math.Sin(Time.realtimeSinceStartup));
        scan();
    }

    private static float RandomGaussian(float minValue = 0.0f, float maxValue = 1.0f)
     {
         float u, v, S;
     
         do
         {
             u = 2.0f * UnityEngine.Random.value - 1.0f;
             v = 2.0f * UnityEngine.Random.value - 1.0f;
             S = u * u + v * v;
         }
         while (S >= 1.0f);
     
         // Standard Normal Distribution
         float std = u * Mathf.Sqrt(-2.0f * Mathf.Log(S) / S);
     
         // Normal Distribution centered between the min and max value
         // and clamped following the "three-sigma rule"
         float mean = (minValue + maxValue) / 2.0f;
         float sigma = (maxValue - mean) / 3.0f;
         return Mathf.Clamp(std * sigma + mean, minValue, maxValue);
     }
}
