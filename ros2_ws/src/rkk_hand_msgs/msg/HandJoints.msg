# Best-effort, keep_last(1); one message per solved frame.

std_msgs/Header header

uint8 HAND_UNKNOWN=0
uint8 HAND_LEFT=1
uint8 HAND_RIGHT=2
uint8 hand

uint32 device_id
uint64 timestamp_us            # source glove clock, not header.stamp
geometry_msgs/Pose[26] joints  # XrHandJointEXT order, matches HandDescription.joint_names
