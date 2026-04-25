# Chunk of a LiDAR scan, compressed to uint16 millimetres to fit in one UDP
# frame under the 2048 B micro-ROS MTU. Chunks with matching scan_id reassemble
# into a single sensor_msgs/LaserScan on the host. 0 in ranges_mm means "no
# return" and maps to inf on reassembly.

std_msgs/Header header
uint16  scan_id
uint8   chunk_index
uint8   chunk_count
float32 angle_min
float32 angle_max
float32 angle_increment
float32 time_increment
float32 scan_time
float32 range_min
float32 range_max
uint16[] ranges_mm
