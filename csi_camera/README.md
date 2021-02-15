# CSI Camera Package for ROS

CSI Camera is a tiny low cost camera which can be used for computer vision.
( Ex: PiCam etc)

### What it solves 
When camera is accessed remotely using ROS there is a huge delay in video stream. Thus this package can be used to access video stream without any huge delay

## Dependencies
```
sudo apt-get install ros-melodic-image-transport-plugins
```

## Instruction

### For NavQ Companion Computer with google coral camera 
Switch to navq_csi branch 

### On Companion Computer (Onboard) 
```
roslaunch csi_camera csi_camera.launch
```
#### Launches following nodes:

1. csi_camera_node.py 
which takes in video stream using opencv/gstreamer and converts it to ros_msg using cv2_bridge.

```
Subscribed to
- None

Publishes to 
- /csi_camera/image
```

2. image_transport node 
which compress the csi_camera images 

```
Subscribed to
- /csi_camera/image

Publishes to 
- /rgb_republish
```

### On Remote PC
```
roslaunch csi_camera decompress.launch
```
Here we use image_transport node to decompress the compressed images from /rgb_republish topic to /rgb_raw

NOTE: Use Rviz to view images on /rgb_raw, to check delay

```
Subscribed To
- /rgb_republish

Publishes To
- /rgb_raw
```
