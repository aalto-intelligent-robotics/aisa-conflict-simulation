# ROS + Unity

1. Install Unity and ROS according to the tutorials in https://github.com/Unity-Technologies/Unity-Robotics-Hub
2. Start Unity, open project and make sure the settings of ROS (Robotics tab in upper menu bar) are ok (ROS1, check IP - IP depends on the location of your ROSCORE)
3. Start TCP Endpoint with `rosrun ros_tcp_endpoint default_server_endpoint.py`
4. Now ROS and Unity are communicating (the arrows in the Unity application GUI should be colored in blue)
5. Publish your control input, so the simulator can use it `rosrun unity_robotics_demo WASP_ackermann_publisher.py`
6. To control the car with a simple lane detection model, run: `rosrun unity_robotics_demo control_from_lane_detection.py`


# Setup

## ROS TCP Endpoint

Download the code for ROS TCP Endpoint in the current folder.

```
git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint
```

The launch file *robo_demo.launch* will launch the node *server_endpoint* from the package *ros_tcp_endpoint* (instructions below).

## Docker

The folder `Docker` contains `Dockerfile` and scripts to install, start and remove the Docker container.

## Setup Paths

* In file `Docker/start_docker.sh`, setup the path to home folder (this project: wasp_ros) and the corresponding Docker folder.
The Docker folder will link to the src folder in the Catkin workspace, so the Docker path should end with `/src`

* In the control Python file `control_from_lane_detection.py`, setup the variables `MODEL_PATH` and `VIZ_SAVE_PATH`.
These are paths in Docker.
 

## Run

We are running these scripts in Docker with ROS Noetic. 

* Build Docker with the script `dockers/install_docker.sh`

* Start Docker with the script `dockers/start_docker.sh`

* In the Docker container, start tmux.

* In the first tab, start `roscore`

* In the second tab, execute:

```
cd [docker_folder] # The same folder specified in the `start_docker.sh` script, without src.

catkin_make

source devel/setup.bash

sh src/launch.sh # run the launch file `launch/robo_demo.launch`
```
