/**
* @file  	vision_att_node.cpp
* @brief 	example: subscribe pixhawk attitude msg for vision using
* @author	huang li long <huanglilongwk@outlook.com>
*/

#include <ros/ros.h>
#include <mission/Attitude.h>

mission::Attitude att_msg;
void att_cb(const mission::Attitude::ConstPtr& msg){
    att_msg = *msg;
	ROS_INFO("Attitude roll: %f", att_msg.roll);
	// TODO with att_msg
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision_att_node");
    ros::NodeHandle nh;

    ros::Subscriber att_sub = nh.subscribe<mission::Attitude>("mavros/attitude", 10, att_cb);

    ros::Rate rate(20.0);

    while(ros::ok()){
	
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}