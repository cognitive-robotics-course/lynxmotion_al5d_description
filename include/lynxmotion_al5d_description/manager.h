#include <cstring>
#include <vector>
#include <thread>
#include <ros/ros.h>
#include "lynxmotion_al5d_description/brick.h"
#include "lynxmotion_al5d_description/Clear.h"
#include "lynxmotion_al5d_description/Reset.h"
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
    bool clear(lynxmotion_al5d_description::Clear::Request &req,
                lynxmotion_al5d_description::Clear::Response &res);
    bool reset(lynxmotion_al5d_description::Reset::Request &req,
                lynxmotion_al5d_description::Reset::Response &res);
    int brickIndex(std::string name);
private:
    std::vector<Brick*> bricks;
    ros::NodeHandle nh;
};

