/**
* @file     : mission_node.cpp
* @brief    : mission example node
* @author   : huang li long <huanglilongwk@outlook.com>
* @time     : 2016/08/26
*/

#include <mission/mission.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mission_node");
    mission::Mission mission;
    mission.mission_main();
    return 0;
}