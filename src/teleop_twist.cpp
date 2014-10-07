#include <ardrone_autonomy/teleop_twist.h>
#include "ardrone_autonomy/LedAnim.h"
#include "utils/ardrone_date.h"

#include <geographic_msgs/KeyValue.h>
#include <vector>

#include <android/log.h>
void blog2(const char *msg, ...) {
    va_list args;
    va_start(args, msg);
    __android_log_vprint(ANDROID_LOG_INFO, "ARDRONE_DRIVER_OBJ_ANDROID", msg, args);
    va_end(args);
}

#define _RAD2DEG 57.2957184819

inline float max(float a, float b) { return a > b ? a : b; }
inline float min(float a, float b) { return a < b ? a : b; }
inline bool in_rangef(float a, float _min, float _max) {return a >= _min && a <= _max; } //inclusive
inline bool in_range(int a, int _min, int _max) {return a >= _min && a <= _max; } //inclusive

bool needs_takeoff = false;
bool needs_land = false;
bool needs_reset = false;
geometry_msgs::Twist cmd_vel;
float old_left_right = -10.0;
float old_front_back = -10.0;
float old_up_down = -10.0;
float old_turn = -10.0;

int cam_state = DEFAULT_CAM_STATE; // 0 for forward and 1 for vertical, change to enum later
int set_navdata_demo_value = DEFAULT_NAVDATA_DEMO; 
int32_t detect_enemy_color = ARDRONE_DETECTION_COLOR_ORANGE_YELLOW;
int32_t detect_dtype = CAD_TYPE_MULTIPLE_DETECTION_MODE;
int32_t detect_hori_type = TAG_TYPE_MASK(TAG_TYPE_SHELL_TAG_V2);
int32_t detect_vert_type = TAG_TYPE_MASK(TAG_TYPE_BLACK_ROUNDEL);
int32_t detect_indoor_hull = 0;
int32_t detect_disable_placeholder = 0;
int32_t detect_enable_placeholder = 1;

const LED_ANIMATION_IDS ledAnimMap[14] = {
	BLINK_GREEN_RED, BLINK_GREEN, BLINK_RED, BLINK_ORANGE,
	SNAKE_GREEN_RED, FIRE, STANDARD, RED, GREEN, RED_SNAKE,BLANK,
	LEFT_GREEN_RIGHT_RED, LEFT_RED_RIGHT_GREEN, BLINK_STANDARD};

//ros service callback to set the camera channel
bool setCamChannelCallback(ardrone_autonomy::CamSelect::Request& request, ardrone_autonomy::CamSelect::Response& response)
{
    const int _modes = (IS_ARDRONE1) ? 4 : 2;
    cam_state = request.channel % _modes;
    ARDRONE_TOOL_CONFIGURATION_ADDEVENT (video_channel, &cam_state, NULL);
    fprintf(stderr, "\nSetting camera channel to : %d.\n", cam_state);
    response.result = true;
    return true;
}
// ros service callback function for toggling Cam
bool toggleCamCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    const int _modes = (IS_ARDRONE1) ? 4 : 2;
    cam_state = (cam_state + 1) % _modes;
    ARDRONE_TOOL_CONFIGURATION_ADDEVENT (video_channel, &cam_state, NULL);
    fprintf(stderr, "\nSetting camera channel to : %d.\n", cam_state);
    return true;
}

// ros service callback to turn on and off camera recording
bool setRecordCallback(ardrone_autonomy::RecordEnable::Request &request, ardrone_autonomy::RecordEnable::Response& response)
{
  char record_command[ARDRONE_DATE_MAXSIZE + 64];
  int32_t new_codec;

  if( request.enable == true ) {
    char date[ARDRONE_DATE_MAXSIZE];
    time_t t = time(NULL);
    // For some reason the linker can't find this, so we'll just do it manually, cutting and pasting
    //    ardrone_time2date(t, ARDRONE_FILE_DATE_FORMAT, date);
    strftime(date, ARDRONE_DATE_MAXSIZE, ARDRONE_FILE_DATE_FORMAT, localtime(&t));
    snprintf(record_command, sizeof(record_command), "%d,%s", USERBOX_CMD_START, date);
    new_codec = MP4_360P_H264_720P_CODEC;
  } else {
    snprintf(record_command, sizeof(record_command), "%d", USERBOX_CMD_STOP );
    new_codec = H264_360P_CODEC;
  }

  vp_os_mutex_lock(&twist_lock);
  ARDRONE_TOOL_CONFIGURATION_ADDEVENT (video_codec, &new_codec, NULL );
  ARDRONE_TOOL_CONFIGURATION_ADDEVENT (userbox_cmd, record_command, NULL );
  vp_os_mutex_unlock(&twist_lock);

  response.result = true;
  return true;
}    

bool setLedAnimationCallback(ardrone_autonomy::LedAnim::Request& request, ardrone_autonomy::LedAnim::Response& response)
{
    LED_ANIMATION_IDS anim_id = ledAnimMap[request.type % 14]; // Don't trick me
    vp_os_mutex_lock(&twist_lock);
    ardrone_at_set_led_animation(anim_id, (float) fabs(request.freq), (uint32_t) abs(request.duration));
    vp_os_mutex_unlock(&twist_lock);
    response.result = true;
    return true;
}

bool setFlightAnimationCallback(ardrone_autonomy::FlightAnim::Request &request, ardrone_autonomy::FlightAnim::Response &response)
{
    char param[20];
    const int anim_type = request.type % ARDRONE_NB_ANIM_MAYDAY;
    const int anim_duration = (request.duration > 0) ? request.duration : MAYDAY_TIMEOUT[anim_type];
    snprintf(param, sizeof (param), "%d,%d", anim_type, anim_duration);
    vp_os_mutex_lock(&twist_lock);
    ARDRONE_TOOL_CONFIGURATION_ADDEVENT(flight_anim, param, NULL);
    vp_os_mutex_unlock(&twist_lock);
    response.result = true;
    return true;
}

bool flatTrimCallback(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
{
    vp_os_mutex_lock(&twist_lock);
    ardrone_at_set_flat_trim();
    vp_os_mutex_unlock(&twist_lock);
    fprintf(stderr, "\nFlat Trim Set.\n");
    blog2("\nFlat Trim Set.\n");
}

void setAutomousFlight(const bool enable) {
    bool_t _e = (bool_t) enable;
    vp_os_mutex_lock(&twist_lock);
    ARDRONE_TOOL_CONFIGURATION_ADDEVENT(flying_camera_enable, &_e, NULL);
    vp_os_mutex_unlock(&twist_lock);
    fprintf(stderr, "\nSet Autonomouse Flight to %s\n", enable ? "ON" : "OFF");
    blog2("\nSet Autonomouse Flight to %s\n", enable ? "ON" : "OFF");
}

bool setAutomousFlightCallback(ardrone_autonomy::RecordEnable::Request &request, ardrone_autonomy::RecordEnable::Response &response){
    setAutomousFlight(request.enable);
    response.result = true;
    return true;
}

bool setGPSTargetWayPointCallback(ardrone_autonomy::SetGPSTarget::Request &request, ardrone_autonomy::SetGPSTarget::Response &response){

    //"10000,0,492767188,-1229157891,994,165,165,525000,0,0"
    //"10000,0,lat,long,alt,vx,vy,525000,orientation(degrees*100),0"

    char param_str[255];
    long int lat = 0, lon = 0;
    int alt = 0, v = 0, orientation = 0;

    if (
            in_rangef(request.target.position.latitude, -90.0, 90.0) &&
            in_rangef(request.target.position.longitude, -180.0, 180.0) &&
            in_rangef(request.target.position.altitude, 0.0, 1000.0) // Don't be crazy though!
        )
    {
        lat = (long int) round(request.target.position.latitude * 1.0e7);
        lon = (long int) round(request.target.position.longitude * 1.0e7);
        alt = (int) round(request.target.position.altitude * 1000.0); //mm
    } else {
        fprintf(stderr, "Invalid value for latitude, longitude or altitude.\n");
        blog2("Invalid value for latitude, longitude or altitude.\n");
        return false;
    }

    // Set default values
    v = 500;
    orientation = 0;

    for(int i = 0; i < request.target.props.size(); i++){
        if(request.target.props[i].key == "velocity"){
            v = (int) round(atof(request.target.props[i].value.c_str()) * 1000.0); // mm/s
            if (!in_range(v, 0, 10000)) { // max: 10m/s
                fprintf(stderr, "Requested velocity is not in range: %d\n", v);
                blog2("Requested velocity is not in range: %d\n", v);
                return false;
            }
        }else if(request.target.props[i].key == "orientation"){
            orientation = (int) round(atof(request.target.props[i].value.c_str()) * _RAD2DEG * 100.0); // degrees
            if (!in_range(orientation, -36000, 36000)) { // max: 360 degrees
                fprintf(stderr, "Requested orientation is not in range: %d\n", orientation);
                blog2("Requested orientation is not in range: %d\n", orientation);
                return false;
            }
        }
    }

    sprintf(param_str,"%d,%d,%ld,%ld,%d,%d,%d,%d,%d,%d",
            10000,
            0,
            lat,
            lon,
            alt,
            v,
            v,
            525000,
            orientation,
            0
            );

    vp_os_mutex_lock(&twist_lock);
    ARDRONE_TOOL_CONFIGURATION_ADDEVENT(flying_camera_mode, param_str, NULL);
    vp_os_mutex_unlock(&twist_lock);
    fprintf(stderr, "\nSet GPS WayPoint \"%s\"\n", param_str);
    blog2("\nSet GPS WayPoint \"%s\"\n", param_str);
    response.result = true;

    return true;
}

void cmdVelCallback(const geometry_msgs::TwistConstPtr &msg)
{
    vp_os_mutex_lock(&twist_lock);
    // Main 4DOF
    cmd_vel.linear.x  = max(min(-msg->linear.x, 1.0), -1.0);
    cmd_vel.linear.y  = max(min(-msg->linear.y, 1.0), -1.0);
	cmd_vel.linear.z  = max(min(msg->linear.z, 1.0), -1.0);
	cmd_vel.angular.z = max(min(-msg->angular.z, 1.0), -1.0);
    // These 2DOF just change the auto hover behaviour
    // No bound() required
    cmd_vel.angular.x = msg->angular.x;
    cmd_vel.angular.y = msg->angular.y;
    vp_os_mutex_unlock(&twist_lock);
}

void landCallback(const std_msgs::Empty &msg)
{
    vp_os_mutex_lock(&twist_lock);
    needs_land = true;
    vp_os_mutex_unlock(&twist_lock);
}

void resetCallback(const std_msgs::Empty &msg)
{	
    vp_os_mutex_lock(&twist_lock);
    needs_reset = true;
    vp_os_mutex_unlock(&twist_lock);
}

void takeoffCallback(const std_msgs::Empty &msg)
{
    vp_os_mutex_lock(&twist_lock);
    needs_takeoff = true;
    vp_os_mutex_unlock(&twist_lock);
}

C_RESULT open_teleop(void)
{
	return C_OK;
}

C_RESULT update_teleop(void)
{
	// This function *toggles* the emergency state, so we only want to toggle the emergency
	// state when we are in the emergency state (because we want to get out of it).
    vp_os_mutex_lock(&twist_lock);
    if (needs_reset)
    {
        ardrone_tool_set_ui_pad_select(1);
        needs_reset = false;
    }
    else if (needs_takeoff)
    {
        ardrone_tool_set_ui_pad_start(1);
        needs_takeoff = false;
    }
    else if (needs_land)
    {
        ardrone_tool_set_ui_pad_start(0);
        needs_land = false;
    }
    else
    {

        float left_right = (float) cmd_vel.linear.y;
        float front_back = (float) cmd_vel.linear.x;
        float up_down = (float) cmd_vel.linear.z;
        float turn = (float) cmd_vel.angular.z;
        
        bool is_changed = !(
                (fabs(left_right - old_left_right) < _EPS) && 
                (fabs(front_back - old_front_back) < _EPS) && 
                (fabs(up_down - old_up_down) < _EPS) && 
                (fabs(turn - old_turn) < _EPS)
                );
        
        // These lines are for testing, they should be moved to configurations
        // Bit 0 of control_flag == 0: should we hover?
        // Bit 1 of control_flag == 1: should we use combined yaw mode?
        
        int32_t control_flag = 0x00;
        int32_t combined_yaw = 0x00;
        
        // Auto hover detection based on ~0 values for 4DOF cmd_vel
        int32_t hover = (int32_t)
                (
                (fabs(left_right) < _EPS) && 
                (fabs(front_back) < _EPS) && 
                (fabs(up_down) < _EPS) && 
                (fabs(turn) < _EPS) &&
                // Set angular.x or angular.y to a non-zero value to disable entering hover
                // even when 4DOF control command is ~0
                (fabs(cmd_vel.angular.x) < _EPS) &&
                (fabs(cmd_vel.angular.y) < _EPS)
                );

        control_flag |= ((1 - hover) << 0);
        control_flag |= (combined_yaw << 1);
        //ROS_INFO (">>> Control Flag: %d", control_flag);
        
        old_left_right = left_right;
        old_front_back = front_back;
        old_up_down = up_down;
        old_turn = turn;
        //is_changed = true;
        if ((is_changed) || (hover))
        {
            ardrone_tool_set_progressive_cmd(control_flag, left_right, front_back, up_down, turn, 0.0, 0.0);
        }

    }
    vp_os_mutex_unlock(&twist_lock);
	return C_OK;
}

C_RESULT close_teleop(void)
{
	return C_OK;
}

input_device_t teleop = {
	"Teleop",
	open_teleop,
	update_teleop,
	close_teleop
};

