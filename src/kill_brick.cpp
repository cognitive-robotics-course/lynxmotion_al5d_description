#include <ros/ros.h>
#include "lynxmotion_al5d_description/KillBrick.h"

using namespace std;

int main (int argc, char **argv) 
{
    if (argc != 2) 
    {
        printf("Usage: rosrun lynxmotion_al5d_description kill_brick name\n");
        exit(1);
    }

    ros::init(argc, argv, "kill_lego_brick");
    ros::NodeHandle nh;
    ros::service::waitForService("/lynxmotion_al5d/kill_brick");
    ros::ServiceClient client = nh.serviceClient<lynxmotion_al5d_description::KillBrick>("/lynxmotion_al5d/kill_brick");

    lynxmotion_al5d_description::KillBrick srv;    
    srv.request.name = argv[1];

    if (client.call(srv))
    {
        if (srv.response.result)
        {
            ROS_INFO("Brick deleted successfully");
        }
        else
        {
            ROS_WARN("%s", srv.response.message.c_str());
        }
    }
    else
    {
        ROS_ERROR("Failed to call the service");
        return 1;
    }
    
    return 0;
}
