#include "mission.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_node");
    mission::Mission mission;
    mission.mission_main();
    return 0;
}