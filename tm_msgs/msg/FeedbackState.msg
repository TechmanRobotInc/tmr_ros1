
Header header

# Connection Status

bool is_svr_connected
bool is_sct_connected

# Movement

float64[] joint_pos
float64[] joint_vel
float64[] joint_tor
float64[] tool0_pose
float64[] tool_pose
float64[] tcp_speed
float64[] tcp_force

# Status

bool robot_link

bool robot_error

bool project_run
bool project_pause

bool safetyguard_a
bool e_stop

bool camera_light

int32 error_code

int32 project_speed
int32 ma_mode
int32 robot_light

# IO

bool[] cb_digital_output
bool[] cb_digital_input
float32[] cb_analog_output
float32[] cb_analog_input
bool[] ee_digital_output
bool[] ee_digital_input
float32[] ee_analog_output
float32[] ee_analog_input

# String

string error_content

