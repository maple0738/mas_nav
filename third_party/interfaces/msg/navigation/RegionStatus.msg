std_msgs/Header header
uint32 trajectory_id
uint8 terrain_label
uint8 mode
uint8 phase
float32 path_progress
float32 speed_limit
# LIO heading feedback to the nearer path axis (+/-X); the sign is not a
# requested drive direction. This is not a chassis-alignment acknowledgement.
float32 gimbal_heading_error_deg
bool gimbal_heading_ready
