# Camera Drivers

This directory contains camera-related ROS 2 packages used by the workspace.

## USB Web Camera

Use the ROS 2 `usb_cam` package from apt instead of maintaining a local USB
camera driver package in this workspace.

Install it for the active ROS 2 distribution:

```bash
sudo apt update
sudo apt install ros-$ROS_DISTRO-usb-cam
```

Check that Linux can see the USB camera:

```bash
ls /dev/video*
v4l2-ctl --list-devices
```

If `v4l2-ctl` is not installed:

```bash
sudo apt install v4l-utils
```

Source ROS 2 and this workspace:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
```

Start the camera node:

```bash
ros2 run usb_cam usb_cam_node_exe --ros-args \
  -p video_device:=/dev/video0 \
  -p image_width:=640 \
  -p image_height:=480 \
  -p framerate:=30.0
```

In another terminal, confirm that image topics are published:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash

ros2 topic list | grep image
ros2 topic hz /image_raw
ros2 topic echo /camera_info --once
```

## Quick rqt Image Test

Install the image viewer if needed:

```bash
sudo apt install ros-$ROS_DISTRO-rqt-image-view
```

Run the viewer:

```bash
ros2 run rqt_image_view rqt_image_view
```

Select `/image_raw` from the topic dropdown. If the camera is working, the live
USB camera image should appear in the viewer.

If no image appears, verify the device path with `v4l2-ctl --list-devices` and
retry with the correct `video_device`, such as `/dev/video1`.
