#include <cstring>
#include <vector>
#include <thread>
#include <ros/ros.h>
#include "lynxmotion_al5d_description/brick.h"
#include "std_srvs/Empty.h"
#include "lynxmotion_al5d_description/SpawnBrick.h"
#include "lynxmotion_al5d_description/KillBrick.h"
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/DeleteModel.h"
#include "std_msgs/Float64MultiArray.h"

#define ROBOT_NAMESPACE "/lynxmotion_al5d"
#define ROBOT_PARAM_SUFFIX "_lego_description"

class Manager
{
public:
    Manager(ros::NodeHandle& n);
    static int numSpawned;
    bool spawnBrick(lynxmotion_al5d_description::SpawnBrick::Request &req,
                    lynxmotion_al5d_description::SpawnBrick::Response &res);
    bool killBrick(lynxmotion_al5d_description::KillBrick::Request &req,
                    lynxmotion_al5d_description::KillBrick::Response &res);
    bool clear(std_srvs::Empty::Request &req,
                std_srvs::Empty::Response &res);
    bool reset(std_srvs::Empty::Request &req,
                std_srvs::Empty::Response &res);
    int brickIndex(std::string name);
private:
    std::vector<Brick*> bricks;
    ros::NodeHandle nh;
};

