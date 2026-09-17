# One atomic, normalized vision snapshot.

std_msgs/Header header
uint32 schema_version
uint64 frame_sequence
string source
string coordinate_space
string finish_edge
float64 conveyor_width_m
float64 conveyor_length_m
string eye_to_hand_frame_id
string eye_to_hand_calibration
fanucpy_ros2_interfaces/VisionDetection[] detections
