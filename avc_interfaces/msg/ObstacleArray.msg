# PLACEHOLDER until Perception confirms the obstacle message (topic, type and rate).
# Published by Perception on /perception/obstacle_detections: every obstacle known so far.
std_msgs/Header header    # frame_id = map (field ENU, origin at the field center)
Obstacle[] obstacles
