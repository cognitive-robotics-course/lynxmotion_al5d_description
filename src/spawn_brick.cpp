#include <ros/ros.h>
#include "lynxmotion_al5d_description/SpawnBrick.h"
#include <string>

using namespace std;

int main (int argc, char **argv) 
{
    if (argc < 3) 
    {
        printf("Usage: rosrun lynxmotion_al5d_description spawn_brick -c color [-n name] [-x x] [-y y] [-z z] [-P pitch] [-R roll] [-Y yaw]\n");
        exit(1);
    }
    ros::init(argc, argv, "spawn_lego_brick");
    // Declare all the variables
    std::string color, name;
    double x = 0, y = 0, z = 0, roll = 0.00, pitch = 0.00, yaw = 0.00;

    for (int i=1; i < argc; ++i) 
    {
        if (string(argv[i]) == "-c" && i + 1 < argc)
        {
            color = string(argv[++i]);
        }
        if (string(argv[i]) == "-n" && i + 1 < argc)
        {
            name = string(argv[++i]);
        }
        else if (string(argv[i]) == "-x" && i + 1 < argc)
        {
            x = stod(argv[++i]);
        }
        else if (string(argv[i]) == "-y" && i + 1 < argc)
        {
            y = stod(argv[++i]);
        }
        else if (string(argv[i]) == "-z" && i + 1 < argc)
        {
            z = stod(argv[++i]);
        }
        else if (string(argv[i]) == "-R" && i + 1 < argc)
        {
            roll = stod(argv[++i]);
        }
        else if (string(argv[i]) == "-P" && i + 1 < argc)
        {
            pitch = stod(argv[++i]);
        }
        else if (string(argv[i]) == "-Y" && i + 1 < argc)
        {
            yaw = stod(argv[++i]);
        }
    }
    // Now we have all the parameters and can now call the service
    
    ros::NodeHandle nh;
    ros::service::waitForService("/lynxmotion_al5d/spawn_brick");
    ros::ServiceClient client = nh.serviceClient<lynxmotion_al5d_description::SpawnBrick>("/lynxmotion_al5d/spawn_brick");
    lynxmotion_al5d_description::SpawnBrick srv;

    srv.request.name = name;    
    srv.request.color = color;
    srv.request.pose.position.x = x;
    srv.request.pose.position.y = y;
    srv.request.pose.position.z = z;
    srv.request.pose.orientation.roll = roll;
    srv.request.pose.orientation.pitch = pitch;
    srv.request.pose.orientation.yaw = yaw;

    if (client.call(srv))
    {
        ROS_INFO("Spawned brick [%s] of color [%s] at position (%.2f %.2f %.2f %.2f %.2f %.2f)", srv.response.name.c_str(), color.c_str(), x, y, z, roll, pitch, yaw);
    }
    else
    {
        ROS_ERROR("Failed to call the service");
        return 1;
    }
    
    return 0;
}
