# One normalized object reported by an external vision provider.

int64 track_id
string identity_source
string label
float32 confidence
bool is_danger
bool geometry_valid

# Center in the projected conveyor image.
bool projected_center_valid
float64 projected_u_px
float64 projected_v_px

# Calibrated 2D eye-to-hand result. The array message identifies its frame and
# calibration. No Z coordinate is provided by this affine calibration.
bool eye_to_hand_valid
float64 eye_to_hand_x_mm
float64 eye_to_hand_y_mm

# Conveyor-plane coordinates in metres. Read them only when geometry_valid is
# true. The array header identifies the frame.
float64 center_x_m
float64 center_s_m
float64 width_m
float64 length_m
string size_profile
