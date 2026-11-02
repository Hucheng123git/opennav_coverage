string mode "UNKNOWN"  # BOUSTROPHEDON, SNAKE, SPIRAL, CUSTOM, CONCENTRIC

# Specific mode setting
uint16 spiral_n 4  # If mode=SPIRAL, this is the number of swaths to spiral
uint16[] custom_order  # If mode=CUSTOM, this is the order of swaths to set. Must be specified.

# CONCENTRIC: distance in meters between successive inward boundary offsets.
# Zero uses default_contour_spacing; a zero server default uses operation_width.
# This generates boundary contours, including holes, instead of parallel swaths.
float64 contour_spacing 0.0
