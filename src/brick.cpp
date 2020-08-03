#include "lynxmotion_al5d_description/brick.h"

/* 
    Converts an angle in degrees to radians.
    ---
    Returns: The angle in radians
*/
double radians(double degrees)
{
    return degrees / (180.0 / M_PI);
}

/* 
    Converts an angle in radians to degrees.
    ---
    Returns: The angle in degrees
*/
double degrees(double radians)
{
    return radians * (180.0 / M_PI);
}

geometry_msgs::Quaternion quaternionFromRPY(double roll, double pitch, double yaw)
{
    geometry_msgs::Quaternion out;
    tf2::Quaternion quat;
    quat.setRPY(roll, pitch, yaw);
    quat.normalize();
    out.x = quat[0];
    out.y = quat[1];
    out.z = quat[2];
    out.w = quat[3];
    return out;
}

RPY RPYFromQuaternion(geometry_msgs::Quaternion q)
{
    RPY values;
    tf2::Quaternion quat(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3(quat).getRPY(values.roll, values.pitch, values.yaw);
    return values;
}

Brick::Brick(ros::NodeHandle& n, std::string _color, std::string _name, double _x, double _y, double _z, double _roll, double _pitch, double _yaw)
{
    // Compute the pose object
    nh = n;
    pose.position.x = _x;
    pose.position.y = _y;
    pose.position.z = _z;
    pose.orientation = quaternionFromRPY(_roll, _pitch, _yaw);
    
    std::cout << pose << std::endl;
    color = _color;
    name = _name;

    // Setting publisher, subscribers and services
    pose_pub = nh.advertise<lynxmotion_al5d_description::Pose>("lynxmotion_al5d/" + name + "/pose", 1000);
    teleport_absolute_srv = nh.advertiseService("/lynxmotion_al5d/" + name + "/teleport_absolute", &Brick::teleportAbsolute, this);
    teleport_relative_srv = nh.advertiseService("/lynxmotion_al5d/" + name + "/teleport_relative", &Brick::teleportRelative, this);
    pose_sub = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1000, &Brick::updatePose, this);
    // Create a timer to send the pose frequently
    timer = nh.createTimer(ros::Duration(0.1), &Brick::publishPose, this);
    timer.start();
}

Brick::~Brick()
{
    // Shutdowns all the nodes started by this "brick" before it gets out of scope
    timer.stop();
    pose_pub.shutdown();
    teleport_absolute_srv.shutdown();
    teleport_relative_srv.shutdown();
}

void Brick::updatePose(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    // Get new pose if it has changed
    ros::spinOnce();
    for (size_t i = 0; msg->name[i] != ""; ++ i)
    {
        if (msg->name[i] == name)
        {
            pose = msg->pose[i];
            break;
        }
    }
}


void Brick::publishPose(const ros::TimerEvent& event)
{ 
    lynxmotion_al5d_description::Pose msg;
    msg.position = pose.position;
    RPY rpy = RPYFromQuaternion(pose.orientation);
    msg.orientation.roll = rpy.roll;
    msg.orientation.pitch = rpy.pitch;
    msg.orientation.yaw = rpy.yaw;
    pose_pub.publish(msg);
}

std::string Brick::getColor()
{
    return this->color;
}

std::string Brick::getName()
{
    return this->name;
}

geometry_msgs::Pose& Brick::getPose()
{
    return this->pose;
}

void Brick::setPose(geometry_msgs::Pose& _newPose)
{
    this->pose = _newPose;
} 

bool Brick::teleport(geometry_msgs::Pose& pose)
{
    ros::service::waitForService("/gazebo/set_model_state");
    gazebo_msgs::SetModelState srv;
    gazebo_msgs::ModelState modelState;
    modelState.model_name = name;
    modelState.pose = pose;
    srv.request.model_state = modelState;
    return ros::service::call("/gazebo/set_model_state", srv); 
}

bool Brick::teleportAbsolute(lynxmotion_al5d_description::TeleportAbsolute::Request &req, lynxmotion_al5d_description::TeleportAbsolute::Response &res)
{
    // Build a new object and use its pose for the operation
    geometry_msgs::Pose goal;
    goal.position.x = req.pose.position.x;
    goal.position.y = req.pose.position.y;
    goal.position.z = req.pose.position.z;
    goal.orientation = quaternionFromRPY(req.pose.orientation.roll, req.pose.orientation.pitch, req.pose.orientation.yaw);
    return teleport(goal);
}

bool Brick::teleportRelative(lynxmotion_al5d_description::TeleportRelative::Request &req, lynxmotion_al5d_description::TeleportRelative::Response &res)
{
    geometry_msgs::Pose goal = pose;
    goal.position.x += req.pose.position.x;
    goal.position.y += req.pose.position.y;
    goal.position.z += req.pose.position.z;
    
    RPY rpy = RPYFromQuaternion(pose.orientation);
    rpy.roll += req.pose.orientation.roll;
    rpy.pitch += req.pose.orientation.pitch;
    rpy.yaw += req.pose.orientation.yaw;

    goal.orientation = quaternionFromRPY(rpy.roll, rpy.pitch, rpy.yaw);

    return teleport(goal);
}
