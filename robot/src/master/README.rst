master
=======

This node makes sure that the VR data is received from the headset is 
valid (in correct mode) and published to the rest of the system.

Subscribed topics:
^^^^^^^^^^^^^^^^^^
* _vr_data (VRData): The untrusted VR drive data received from the headset.
* _vr_hand (VRHand): The untrusted VR hand data received from the headset.
* _vr_mode (VRMode): The untrusted VR mode data received from the headset.


Published topics:
^^^^^^^^^^^^^^^^^
* vr_data (VRData): The VR drive data received from the headset.
* vr_hand (VRHand): The VR hand data received from the headset.
* vr_mode (VRMode): The VR mode data received from the headset.
