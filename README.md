# Capstone Project – Programming A Real Self-Driving Car

## Team RobotCar

![Header Image Carla](imgs/header.jpeg)
###### *Udacity. "Carla" Udacity.com*

In the final project of the Udacity Self-Driving Car Nanodegree, we were tasked with implementing core autonomous subsystem functionality to allow Udacity’s Self-Driving Lincoln MKZ to autonomously navigate around a test track. This test will verify the vehicle's ability to follow a path of provided waypoints and stop safely at red illuminated traffic lights.

<br />

## Team Members 

| Name |
| :---: |
| Jeremy Matson |
| Mario Lüder |
| Emilio Moyers |
| Yang Sun |

<br />

## Vehicle Subsystems

Three major subsystems were configured to communicate with each other utilizing the Robot Operating System (ROS). Each subsystem is comprised of multiple components to fulfill the greater task.

![ROS Architecture Diagram](imgs/ros_diagram.png)
###### *Udacity. "final-project-ros-graph-v2.png" Udacity.com*

<br />

### Planning

The first subsystem to implement was the Planning subsystem. It consists of the Waypoint Loader and the Waypoint Updater nodes.

#### Waypoint Loader

The Waypoint Loader node (*/waypoint_loader*) loads the initial waypoints for the track that the vehicle will be tested on. These waypoints contain information about the target pose of the vehicle (x, y, and heading) and the target velocity.

<img src="imgs/sim_start.png" width="600" alt="Simulator Start">

#### Waypoint Updater

The Waypoint Updater node (*/waypoint_updater*) is responsible for adjusting the longitudinal velocity component of the waypoints to account for deceleration events. These events are determined by the Control and Perception subsystems.

As braking events are demanded when a RED traffic light is detected, the function implemented in this node starts by calculating the distance of each waypoint from the vehicle to the target stop line for the traffic light. Then, the time required to decelerate to a complete stop is calculated based upon the maximum deceleration rate configured for limiting jerk and passenger discomfort. Finally, the target velocity under braking is calculated and applied to each waypoint. This allows for a linear adaptive braking function that can scale with vehicle velocity.

<img src="imgs/sim_run.png" width="600" alt="Simulator Waypoints">

<br />

### Control

The Control Subsystem was the next to implement. It consists of the Drive-By-Wire (*/twist_controller*) and Waypoint Follower (*/waypoint_follower*) nodes.

#### DBW Node

The DBW node (*/dbw_node.py*) is responsible for providing new proposed linear and angular velocities to allow the vehicle to maintain the path planned by the Waypoint Updater node. It consists of PID controller functions for throttle control (*twist_controller.py, pid.py*), a brake torque calculation, a yaw-controller (*yaw_controller.py*) to adjust heading direction, and a low-pass filter to reduce sensor noise (*lowpass.py*).

The braking function implemented calculates the distance 

<img src="imgs/pid_test.png" width="600" alt="PID Test Waypoint Following">

#### Waypoint Follower

The Waypoint Follower node (*/waypoint_follower.py*) is Autoware open-source code that is responsible for outputting the control commands to the vehicle that have been provided by the DBW node.

<br />

### Perception

The perception subsystem consists of a Traffic Light Detector. Note: The Obstacle Detection node has not been implemented but has been framed for future use.

#### Traffic Light Detector

The Traffic Light Detection node (*/tl_detector*) consists of a light detector, and a classifier. The Light Detector (*tl_detector.py*) subscribes to images published by the vehicle’s forward-facing camera, dynamically adjusts the image processing rate, sends images to the classifier for light state detection (RED, YELLOW, GREEN, or UNKNOWN), and publishes the location of the stop line for the detected stop light for the Planning Subsystem to act upon in the event of a RED light.

It was found that lag can be induced into the simulator as experienced by the vehicle exhibiting a waypoint trail extending behind it while traversing the test track. This is heavily dependent upon machine resources, but can be helped by classifying images at extended intervals or dropping images. In this project, it was decided upon to drop 9 out of every 10 images when the vehicle's position exceeded 100 waypoints of the traffic light stop line, and then this was reduced to 3 out of every 4 images when it was within this threshold.

<img src="imgs/sim_stop.png" width="600" alt="Simulator stopped at red light">

#### Traffic Light Classifier

The Traffic Light Classifier is a TensorFlow model that is fed the forward-facing camera image from the Traffic Light Detector and returns a state of the traffic light if it is found. The model chosen was the **“Single Shot Detection Inception V2”** algorithm, which offers better performance than the **“Single Shot Detection Mobilenet V1”** algorithm, at a slight expense of speed. This model performed very well for our application.

Additional information on the SSD Inception V2 Model can be found at:

[Inception Model](https://arxiv.org/abs/1512.00567 "Inception Model")

![SSD Inception V2 Model Diagram](imgs/ssd_v2.jpeg)
###### *Towards Data Science. "SSD Architecture" TowardsDataScience.com*

##### Classified red stop light (simulator):

<img src="imgs/classify_red.jpg" width="600" alt="Classified red light with high certainty">

##### Classified yellow stop light (simulator):

<img src="imgs/classify_yellow.jpg" width="600" alt="Classified yellow light with high certainty">

##### Classified red stop light (Carla):

<img src="imgs/classify_red_site.jpg" width="600" alt="Classified red light with high certainty">

##### Classified green stop light (Carla):

<img src="imgs/classify_green_site.jpg" width="600" alt="Classified green light with high certainty">

## Results

A video of the simulator run can be found below:
 
[![Simulator Video](https://img.youtube.com/vi/ilKkEYNfy_U/0.jpg)](https://www.youtube.com/watch?v=ilKkEYNfy_U)

<br />

# Installation Instructions

This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

Please use **one** of the two installation options, either native **or** docker installation.

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
To set up port forwarding, please refer to the [instructions from term 2](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77)

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images
