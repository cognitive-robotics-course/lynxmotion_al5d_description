#include <ros/ros.h>
#include <lynxmotion_al5d_description/manager.h>

int main (int argc, char **argv)
{
    ros::init(argc, argv, "spawn_brick_server");
    ros::NodeHandle nh;
    Manager nodeManager(nh);
    Manager::numSpawned = 0;

    char spawnServiceName [64] = "/lynxmotion_al5d/spawn_brick";
    char killServiceName [64] = "/lynxmotion_al5d/kill_brick";
    char teleportAbsoluteServiceName [64] = {};
    char teleportRelativeServiceName [64] = {};
    char resetServiceName [64] = "/lynxmotion_al5d/reset";
    char clearServiceName [64] = "/lynxmotion_al5d/clear";

    char posePublisherName [64] = {};

    ros::ServiceServer spawnService = nh.advertiseService(spawnServiceName, &Manager::spawnBrick, &nodeManager);
    ros::ServiceServer killService = nh.advertiseService(killServiceName, &Manager::killBrick, &nodeManager);
    ros::ServiceServer clearService = nh.advertiseService(clearServiceName, &Manager::clear, &nodeManager);
    ros::ServiceServer resetService = nh.advertiseService(resetServiceName, &Manager::reset, &nodeManager);

    ROS_INFO("Bricks management service loaded!");
    ros::spin();
    
    return 0;
}
