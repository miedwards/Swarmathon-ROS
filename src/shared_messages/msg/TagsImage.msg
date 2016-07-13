
std_msgs/UInt16MultiArray tags
sensor_msgs/Image image
geometry_msgs/Point[] centers

# Each polygon object will contain 4 corners
# of an april tag in no particular order
# in the points array
# ex: polygon[0].points[0].x polygon[0].points[0].y
# The index corresponds to the april tag, so the 0th index
# will be the same april tag as the 0th index in the center 
# polygon object
geometry_msgs/Polygon[] corners
