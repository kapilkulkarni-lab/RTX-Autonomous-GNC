# PLACEHOLDER until Perception confirms this message (topic, type, frame and rate).
# Published by Perception on /perception/target_relative: where the mission's target die is
# relative to the drone, from the downward camera. GNC uses it to center over the die.
std_msgs/Header header
bool valid                    # false: target not in view; ignore offset
geometry_msgs/Point offset    # target minus drone, field ENU meters (x east, y north, z up)
