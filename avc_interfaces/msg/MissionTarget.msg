# Published by GNC (mission_manager) on /mission/target: the active target color and, once
# known, its centroid. QoS: reliable, transient local. Message format is GNC's own placeholder
# until the other teams confirm what they need.
std_msgs/Header header        # frame_id = map (field ENU, origin at the field center)
string color                  # blue | red | yellow | black, '' if none
bool known                    # true once the die's centroid is known
geometry_msgs/Point centroid  # field ENU meters; ignore when known is false
