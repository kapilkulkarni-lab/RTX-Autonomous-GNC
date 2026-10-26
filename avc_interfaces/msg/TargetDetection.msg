# Publication time; map frame uses field ENU metres.
std_msgs/Header header
# Remains true after first observation until camera node restart.
bool found
# Target ground location; ignore when found is false.
geometry_msgs/Point position
