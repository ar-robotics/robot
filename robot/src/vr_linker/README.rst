vr_linker
=========

This is the node which connects the Meta Quest 3 VR headset to the ROS 2 ecosystem.
It hosts a TCP socket server the headset can connect to. 
Data received here gets sent to the ROS2 ecosystem.

**Subscribed topics**

* robot_data (RobotData): The robot data received from the robot, sent to the VR headset over socket.

**Published topics**

* _vr_data (VRData): The VR drive data received from the headset.
* _vr_hand (VRHand): The VR hand data received from the headset.
* _vr_mode (VRMode): The VR mode data received from the headset.
