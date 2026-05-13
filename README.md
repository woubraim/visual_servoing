# Visual Servoing

ROS 2 package for AprilTag-based visual servoing experiments.

This package detects AprilTags from a camera stream, estimates their 3D pose, displays the detections in an OpenCV window, and publishes detected goals for future visual servoing control.

The node supports two camera setups:

- OAK camera through `depthai_ros_driver_v3`
- Intel RealSense camera through `realsense2_camera`

If the robot TF tree is not available, the node still displays the detected tags and publishes their pose in the camera frame. This allows testing the vision pipeline without launching the full robot stack.

## Camera setup

This package can be tested with either an OAK camera or an Intel RealSense camera.

The visual servoing node does not launch the camera by itself. The camera driver must be launched first in a separate terminal.

---

## OAK Camera Installation

The OAK camera is used through the DepthAI ROS driver.

For ROS 2 Humble, the v3 driver package uses the `_v3` suffix, for example:

```text
depthai_ros_driver_v3
```

The official DepthAI ROS v3 documentation is available here:

```text
https://docs.luxonis.com/software-v3/depthai/ros/driver
```

### Install DepthAI ROS driver

```bash
sudo apt update
sudo apt install ros-humble-depthai-ros-v3
```

If the package is not found, enable the ROS testing repository as described in the Luxonis documentation:

```bash
sudo apt install ros2-testing-apt-source
sudo apt update
sudo apt install ros-humble-depthai-ros-v3
```

### Verify OAK camera

Plug in the OAK camera and check that it is detected over USB:

```bash
lsusb | grep -i movidius
```

or:

```bash
lsusb | grep -i luxonis
```

You can also directly test the ROS driver:

```bash
ros2 launch depthai_ros_driver_v3 driver.launch.py
```

### Launch OAK camera for this package

For this visual servoing package, launch the OAK camera with depth disabled and a custom RGB configuration file:

```bash
ros2 launch depthai_ros_driver_v3 driver.launch.py \
  enable_depth:=false \
  params_file:=/home/woubraim/oak_1080.yaml
```

The visual servoing node expects the following topics:

```text
/oak/rgb/image_raw
/oak/rgb/camera_info
```

Check that they exist:

```bash
ros2 topic list | grep oak
```

Check that the image stream is publishing:

```bash
ros2 topic hz /oak/rgb/image_raw
```

Check the camera info:

```bash
ros2 topic echo /oak/rgb/camera_info --once
```

---

## RealSense Camera Installation

The RealSense camera is used through the Intel RealSense SDK and the ROS 2 wrapper.

Official ROS wrapper documentation:

```text
https://github.com/realsenseai/realsense-ros
```

Official librealsense SDK documentation:

```text
https://github.com/IntelRealSense/librealsense
```

If installation fails because of kernel or DKMS issues, the libuvc backend installation can be used:

```text
https://github.com/IntelRealSense/librealsense/blob/master/doc/libuvc_installation.md
```

### Install RealSense SDK

Install the RealSense SDK, also called `librealsense`.

Follow the official installation guide from Intel RealSense:

```text
https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md
```

### Verify RealSense camera

Plug in the camera and run:

```bash
realsense-viewer
```

If the RGB stream appears in `realsense-viewer`, the camera is working correctly.

### Install ROS 2 wrapper

Install the ROS 2 RealSense wrapper:

```bash
sudo apt update
sudo apt install ros-humble-realsense2-camera
```

### Launch RealSense camera for this package

Launch the RealSense camera with RGB at 1920x1080, 30 FPS:

```bash
ros2 launch realsense2_camera rs_launch.py \
  rgb_camera.color_profile:=1920,1080,30
```

The visual servoing node expects the following topics:

```text
/camera/camera/color/image_raw
/camera/camera/color/camera_info
```

Check that they exist:

```bash
ros2 topic list | grep camera
```

Check that the image stream is publishing:

```bash
ros2 topic hz /camera/camera/color/image_raw
```

Check the camera info:

```bash
ros2 topic echo /camera/camera/color/camera_info --once
```

---

## Camera topic summary

| Camera | Image topic | Camera info topic |
|---|---|---|
| OAK | `/oak/rgb/image_raw` | `/oak/rgb/camera_info` |
| RealSense | `/camera/camera/color/image_raw` | `/camera/camera/color/camera_info` |

The selected camera is controlled with the `camera_type` parameter of the visual servoing node:

```bash
-p camera_type:=oak
```

or:

```bash
-p camera_type:=realsense
```
---

## Features

- AprilTag detection using the `tag36h11` family
- Pose estimation from camera intrinsics
- OAK and RealSense camera support
- OpenCV visualization
- SAVE POSE button in the display window
- Publishes detected tag poses on a ROS 2 topic
- Fallback behavior when `base_link` TF is unavailable

---

## Package structure

```text
visual_servoing/
├── include/visual_servoing/
│   └── visual_servoing_display.hpp
├── msg/
│   ├── DetectedGoal.msg
│   └── DetectedGoalArray.msg
├── srv/
│   └── SaveCurrentTagGoal.srv
├── src/
│   ├── visual_servoing_node.cpp
│   └── visual_servoing_display.cpp
├── third_party/
│   └── apriltag/
├── CMakeLists.txt
├── package.xml
└── README.md
## Dependencies

This package requires ROS 2 Humble.

Install common ROS dependencies:

```bash
sudo apt update
sudo apt install ros-humble-cv-bridge ros-humble-tf2-ros ros-humble-tf2-geometry-msgs
```

It also requires OpenCV, Eigen3, and AprilTag.

On this setup, the AprilTag library is expected here:

```text
/usr/local/include/apriltag
/usr/local/lib/libapriltag.so
```

Check that AprilTag is installed:

```bash
ls /usr/local/include | grep apriltag
ls /usr/local/lib | grep apriltag
```

Expected output should contain something like:

```text
apriltag
libapriltag.so
```

---

## Build

From the workspace root:

```bash
cd ~/extender_ws/visual_servoing_ws

source /opt/ros/humble/setup.bash

colcon build --symlink-install

source install/setup.bash
```

---

## Launch camera

### Option 1: OAK camera

Launch the OAK camera with RGB only:

```bash
ros2 launch depthai_ros_driver_v3 driver.launch.py \
  enable_depth:=false \
  params_file:=/home/woubraim/oak_1080.yaml
```

The visual servoing node expects the following topics:

```text
/oak/rgb/image_raw
/oak/rgb/camera_info
```

---

### Option 2: RealSense camera

Launch the RealSense camera:

```bash
ros2 launch realsense2_camera rs_launch.py \
  rgb_camera.color_profile:=1920,1080,30
```

The visual servoing node expects the following topics:

```text
/camera/camera/color/image_raw
/camera/camera/color/camera_info
```

---

## Run visual servoing node

### With OAK camera

```bash
cd ~/extender_ws/visual_servoing_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run visual_servoing visual_servoing_node --ros-args -p camera_type:=oak
```

### With RealSense camera

```bash
cd ~/extender_ws/visual_servoing_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run visual_servoing visual_servoing_node --ros-args -p camera_type:=realsense
```

---

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `camera_type` | string | `oak` | Camera setup to use. Supported values: `oak`, `realsense` |
| `fullscreen_display` | bool | `true` | Opens the OpenCV display in fullscreen |
| `target_frame` | string | `base_link` | Target TF frame used for transformed tag poses |
| `tag_size` | double | `0.024` | AprilTag size in meters |

Example:

```bash
ros2 run visual_servoing visual_servoing_node --ros-args \
  -p camera_type:=oak \
  -p target_frame:=base_link \
  -p tag_size:=0.024 \
  -p fullscreen_display:=true
```

---

## Published topics

### `/visual_servoing/detected_goals`

Type:

```text
visual_servoing/msg/DetectedGoalArray
```

This topic publishes the detected AprilTags and their estimated poses.

Inspect the detections with:

```bash
ros2 topic echo /visual_servoing/detected_goals
```

---

## Service client

The node contains a client for:

```text
/visual_servoing/save_current_tag_goal
```

Type:

```text
visual_servoing/srv/SaveCurrentTagGoal
```

This service is intended to save the pose of the currently visible tag when the `SAVE POSE` button is clicked.

If no service server is running, the display will show:

```text
Service not ready
```

---

## TF behavior

The node tries to transform each detected tag pose from the camera frame to:

```text
base_link
```

or to the frame specified by the `target_frame` parameter.

If the TF transform is missing, for example:

```text
Could not transform from oak_rgb_camera_optical_frame to base_link
```

this is not fatal.

The node will still:

- display the detected tag in the OpenCV image;
- publish the tag pose in the camera frame;
- allow camera-only testing.

This is useful when testing vision without launching the full robot TF tree.

---

## Quick test with OAK

Terminal 1, launch the OAK camera:

```bash
ros2 launch depthai_ros_driver_v3 driver.launch.py \
  enable_depth:=false \
  params_file:=/home/woubraim/oak_1080.yaml
```

Terminal 2, run the node:

```bash
cd ~/extender_ws/visual_servoing_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run visual_servoing visual_servoing_node --ros-args -p camera_type:=oak
```

Terminal 3, check detections:

```bash
source /opt/ros/humble/setup.bash
source ~/extender_ws/visual_servoing_ws/install/setup.bash

ros2 topic echo /visual_servoing/detected_goals
```

---

## Quick test with RealSense

Terminal 1, launch the RealSense camera:

```bash
ros2 launch realsense2_camera rs_launch.py \
  rgb_camera.color_profile:=1920,1080,30
```

Terminal 2, run the node:

```bash
cd ~/extender_ws/visual_servoing_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run visual_servoing visual_servoing_node --ros-args -p camera_type:=realsense
```

Terminal 3, check detections:

```bash
source /opt/ros/humble/setup.bash
source ~/extender_ws/visual_servoing_ws/install/setup.bash

ros2 topic echo /visual_servoing/detected_goals
```

