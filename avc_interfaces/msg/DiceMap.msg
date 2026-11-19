# PLACEHOLDER until Perception confirms the dice map message (topic, type and rate).
# Published by Perception on /perception/dice_map: every die seen so far, with its color and xyz centroid.
# GNC reads it directly and keeps no die memory of its own.
std_msgs/Header header    # frame_id = map (field ENU, origin at the field center)
DiceMapEntry[] dice
