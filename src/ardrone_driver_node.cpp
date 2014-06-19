#include "ardrone_driver.h"
#include "teleop_twist.h"
#include "video.h"
#include <signal.h>

// Load actual auto-generated code to publish full navdata
#define NAVDATA_STRUCTS_SOURCE
#include "NavdataMessageDefinitions.h"
#undef NAVDATA_STRUCTS_SOURCE

void ARDroneDriver::publish_tf()
{
    tf_base_front.stamp_ = ros::Time::now();
    tf_base_bottom.stamp_ = ros::Time::now();
    tf_broad.sendTransform(tf_base_front);
    tf_broad.sendTransform(tf_base_bottom);
}

bool ARDroneDriver::imuReCalibCallback(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
{
    if (!do_caliberation)
    {
        ROS_WARN("Automatic IMU Caliberation is not active. Activate first using `do_imu_caliberation` parameter");
        return false;
    }
    else
    {
        ROS_WARN("Recaliberating IMU, please do not move the drone for a couple of seconds.");
        resetCaliberation();
        return true;
    }
}

void controlCHandler (int signal)
{
    ros::shutdown();
    should_exit = 1;
}

////////////////////////////////////////////////////////////////////////////////
// custom_main
////////////////////////////////////////////////////////////////////////////////

//extern "C" int custom_main(int argc, char** argv)
int main(int argc, char** argv)
{
    C_RESULT res = C_FAIL;
    char * drone_ip_address = NULL;

    // We need to implement our own Signal handler instead of ROS to shutdown
    // the SDK threads correctly.

    ros::init(argc, argv, "ardrone_driver", ros::init_options::NoSigintHandler);

    signal (SIGABRT, &controlCHandler);
    signal (SIGTERM, &controlCHandler);
    signal (SIGINT, &controlCHandler);

    // Now to setup the drone and communication channels
    // We do this here because calling ardrone_tool_main uses an old
    // function initialization and is no longer recommended by parrot
    // I've based this section off the ControlEngine's initialization
    // routine (distributed with ARDrone SDK 2.0 Examples) as well as
    // the ardrone_tool_main function

    // Parse command line for
    // Backward compatibility with `-ip` command line argument
    argc--; argv++;
    while( argc && *argv[0] == '-' )
    {
        if( !strcmp(*argv, "-ip") && ( argc > 1 ) )
        {
            drone_ip_address = *(argv+1);
            printf("Using custom ip address %s\n",drone_ip_address);
            argc--; argv++;
        }
        argc--; argv++;
    }

    // Configure wifi
    vp_com_wifi_config_t *config = (vp_com_wifi_config_t*)wifi_config();

    if(config)
    {

        vp_os_memset( &wifi_ardrone_ip[0], 0, ARDRONE_IPADDRESS_SIZE );

        // TODO: Check if IP is valid
        if(drone_ip_address)
        {
          printf("===================+> %s\n", drone_ip_address);
          strncpy( &wifi_ardrone_ip[0], drone_ip_address, ARDRONE_IPADDRESS_SIZE - 1);
        }
        else
        {
          printf("===================+> %s\n", config->server);
          strncpy( &wifi_ardrone_ip[0], config->server, ARDRONE_IPADDRESS_SIZE - 1);
        }
    }

    while (-1 == getDroneVersion (".", wifi_ardrone_ip, &ardroneVersion))
    {
        printf ("Getting AR.Drone version ...\n");
        vp_os_delay (250);
    }

    // Setup communication channels
    res = ardrone_tool_setup_com( NULL );
    if( FAILED(res) )
    {
        PRINT("Wifi initialization failed. It means either:\n");
        PRINT("\t* you're not root (it's mandatory because you can set up wifi connection only as root)\n");
        PRINT("\t* wifi device is not present (on your pc or on your card)\n");
        PRINT("\t* you set the wrong name for wifi interface (for example rausb0 instead of wlan0) \n");
        PRINT("\t* ap is not up (reboot card or remove wifi usb dongle)\n");
        PRINT("\t* wifi device has no antenna\n");
    }
    else
    {
        // setup the application and user profiles for the driver

        char* appname = (char*) DRIVER_APPNAME;
        char* usrname = (char*) DRIVER_USERNAME;
        ardrone_gen_appid (appname, "2.0", app_id, app_name, APPLI_NAME_SIZE);
        ardrone_gen_usrid (usrname, usr_id, usr_name, USER_NAME_SIZE);

        // and finally initialize everything!
        // this will then call our sdk, which then starts the ::run() method of this file as an ardrone client thread

        res = ardrone_tool_init(wifi_ardrone_ip, strlen(wifi_ardrone_ip), NULL, app_name, usr_name, NULL, NULL, MAX_FLIGHT_STORING_SIZE, NULL);

        while( SUCCEED(res) && ardrone_tool_exit() == FALSE )
        {
            res = ardrone_tool_update();
        }
        res = ardrone_tool_shutdown();
    }
    return SUCCEED(res) ? 0 : -1;
}


