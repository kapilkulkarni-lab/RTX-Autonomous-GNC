# PLACEHOLDER until Perception confirms its heartbeat (topic, type and rate; 5 Hz planned).
# Published by Perception on /perception/heartbeat. GNC judges liveness by message age.
std_msgs/Header header
bool vio_ok          # visual-inertial odometry healthy
bool detector_ok     # dice detector healthy
