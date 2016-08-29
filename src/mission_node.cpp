#include <mission/mission.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mission_node");
    mission::Mission mission;
    mission.mission_main();
    return 0;
}