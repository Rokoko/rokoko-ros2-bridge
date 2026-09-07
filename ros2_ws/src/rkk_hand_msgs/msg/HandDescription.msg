# Latched (reliable + transient_local); published once per hand.

std_msgs/Header header

uint8 HAND_LEFT=0
uint8 HAND_RIGHT=1
uint8 hand

uint32 device_id
string timestamp_epoch   # "device_boot" | "unix_epoch"
string[26] joint_names   # XrHandJointEXT order, no "xr_" prefix
float64[26] joint_radii  # meters
