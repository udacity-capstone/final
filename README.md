# Team TieInFighter

## Team Members:
* Siraj Haque (siraj_79@hotmail.com)
* Amol Sahasrabudhe (amol.sahasrabudhe@gmail.com)
* Jason Kang (jacquestkirk@gmail.com)
* Shobhit K (shobhitkukreti@yahoo.com)
* Daniel Tang (daniel.z.tang@gmail.com)

## Introduction

For this project, our team designed a fully autonomous vehicle system, initially to be tested out on a simulator, and then on Udacity’s real self-driving car. The project can be broken up into 4 parts: (1) traffic light detection, (2) control by drive-by-wire (DBW), (3) Waypoint updater/follower, and (4) system integration.

## Traffic Light Detection

TBD

## Control by DBW

TBD

## Waypoint Updater/Follower

TBD

## System Integration

TBD 
## Appendix

TBD

### Development Environment Setup

To avoid any risk of incompatiblity of final integration and real world testing, our team decide to use Ubuntu 16.04 Xenial Xerus with [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) for our dev environment. DBW is also downloaded from [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros). Finally the simulator is downloaded at [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

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
