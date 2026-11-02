# An ordered set of swaths (if route generated) and turns (if path generated)
# CONCENTRIC represents every straight contour edge as one swath; turns separate
# adjacent edges and connect closed contours, preserving the existing structure.
std_msgs/Header header
opennav_coverage_msgs/Swath[] swaths
nav_msgs/Path[] turns
bool contains_turns
bool swaths_ordered
