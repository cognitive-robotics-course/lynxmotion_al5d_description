#include <ros/ros.h>
#include <string>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <tf2/LinearMath/Quaternion.h>
#include "lynxmotion_al5d_description/TeleportAbsolute.h"
#include "lynxmotion_al5d_description/TeleportRelative.h"
#include <tf2/LinearMath/Matrix3x3.h>

#define PUBLISH_FREQUENCY 10
class Brick {
public:
    Brick(ros::NodeHandle& n, std::string _color, std::string _name, double _x = 0, double _y = 0, double _z = 0, double _roll = 0.00, double _pitch = 0.00, double _yaw = 0.00);
    ~Brick();
    std::string getColor();
    std::string getName();
    geometry_msgs::Pose& getPose();
    void publishPose(const ros::TimerEvent& event);
    void setPose(geometry_msgs::Pose& _newPose);
    bool teleportAbsolute(lynxmotion_al5d_description::TeleportAbsolute::Request &req, lynxmotion_al5d_description::TeleportAbsolute::Response &res);
    bool teleportRelative(lynxmotion_al5d_description::TeleportRelative::Request &req, lynxmotion_al5d_description::TeleportRelative::Response &res);
    bool teleport(geometry_msgs::Pose& pose);
    void updatePose(const gazebo_msgs::ModelStates::ConstPtr& msg);
private:
    std::string color;
    std::string name;
    geometry_msgs::Pose pose;
    ros::NodeHandle nh;
    ros::Publisher pose_pub;
    ros::Subscriber pose_sub;
    ros::ServiceServer teleport_absolute_srv;
    ros::ServiceServer teleport_relative_srv;
    ros::Timer timer;
};

typedef struct RPY
{
    double roll, pitch, yaw;
} RPY;

double degrees (double radians);
double radians (double degrees);
geometry_msgs::Quaternion quaternionFromRPY(double roll, double pitch, double yaw);
RPY RPYFromQuaternion(geometry_msgs::Quaternion quat);
