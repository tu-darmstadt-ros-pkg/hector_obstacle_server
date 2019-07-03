# Unique id. will be assigned
# by the obstacle
string id

int8 shape_type

int8 SHAPE_INVALID = 0
int8 SHAPE_CIRCLE = 1
int8 SHAPE_POLYGON = 2
int8 SHAPE_RECTANGLE = 3
int8 SHAPE_LINE_WITH_ENDPOINTS = 4

# Applies to SHAPE_CIRCLE only
float64 radius

# Applies to SHAPE_RECTANGLE only
geometry_msgs/Pose pose
float64 rect_x
float64 rect_y
float64 rect_z

# Vector of points
# For SHAPE_POLYGON points on polygon shape
# For SHAPE_LINE_WITH_ENDPOINTS two endpoints
geometry_msgs/Point[] points

