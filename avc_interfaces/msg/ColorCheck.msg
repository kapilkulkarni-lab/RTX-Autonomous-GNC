# PLACEHOLDER until Perception confirms this message's format.
# Perception's confirmed die color from BrainChip on the downward camera, on /perception/color_check.
# Perception owns the windowing and the confidence threshold; GNC reads only color_guess and ignores confidence.
std_msgs/Header header
string color_guess      # blue | red | yellow | black
float32 confidence      # 0 to 1
