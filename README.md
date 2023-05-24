# ROS Synchronized Cameras

This package provides a ROS node for synchronizing two USB cameras. The images from the two cameras are captured and synchronized using ROS message filters. This is useful in scenarios where stereo image processing or other multi-camera applications are needed.

## Dependencies

The following ROS packages are needed:

- `usb_cam`: A ROS driver for USB cameras. This driver publishes images as `sensor_msgs/Image` messages, which can be used directly in ROS.
- `cv_bridge`: A ROS library for converting between ROS image messages and OpenCV image objects.
- `message_filters`: A ROS library for processing messages from multiple topics.

In addition, the following Python libraries are needed:

- `rospy`: A Python library for writing ROS nodes.
- `cv2`: OpenCV's Python library, used for image processing.

## Installation

First, clone this repository into your catkin workspace:

```bash
cd ~/catkin_ws/src
git clone https://github.com/yourgithubusername/ros_synchronized_cameras.git
```
Next, install the dependencies:
```bash
sudo apt update
sudo apt install ros-noetic-usb-cam ros-noetic-cv-bridge python3-opencv python3-rospy ros-noetic-common-msgs
```
Please replace noetic with your ROS distribution if you're using a different one.

After installing the dependencies, build your catkin workspace:
```bash
cd ~/catkin_ws
catkin_make
```

## Usage
First, start the USB camera drivers for both cameras. Replace /dev/video0 and /dev/video1 with the device paths for your cameras:

```bash
roslaunch usb_cam usb_cam.launch video_device:=/dev/video0 camera_name:=usb_cam1 image_topic:=image_raw
roslaunch usb_cam usb_cam.launch video_device:=/dev/video1 camera_name:=usb_cam2 image_topic:=image_raw
```

Next, start the sync_cameras node:
```bash
rosrun ros_synchronized_cameras sync_cameras.py
```

The node will display the synchronized images from both cameras in a single window. The node will keep running until you press 'q' to close the window.
