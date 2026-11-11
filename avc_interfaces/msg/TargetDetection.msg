# Publication time; map uses field East, North, Up metres.
std_msgs/Header header
# Remains true after first observation until camera node restart.
bool found
# Target ground location in the header frame; ignore when found is false.
geometry_msgs/Point position
