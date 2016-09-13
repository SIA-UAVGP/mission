/**
* @file  	: custom_msg_node.cpp
* @brief 	: custom mavlink msg test
* @author	: huang li long <huanglilongwk@outlook.com>
* @time		: 2016/09/05
*/

#include <ros/ros.h>
#include <mission/State.h>
#include <mission/Mavros_test_msg.h>
#include <mission/ObstaclePosition.h>

mission::State current_state;
void state_cb(const mission::State::ConstPtr& msg){
    current_state = *msg;
}

mission::Mavros_test_msg msg_test;
void mavros_msg_cb(const mission::Mavros_test_msg::ConstPtr& msg){
	msg_test = *msg;
	ROS_INFO("mavros_msg test: %f", msg_test.test);
}

// mission::ObstaclePosition obs_pos;
// void obs_pos_cb(const mission::ObstaclePosition::ConstPtr& msg){
// 	obs_pos = *msg;
// 	ROS_INFO("obstacle position: %f", obs_pos.obstacle_x);
// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "custom_msg_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mission::State>("mavros/state", 10, state_cb);

	//ros::Subscriber test_msg_sub = nh.subscribe<mission::Mavros_test_msg>("mavros/mavros_test_msg", 10, mavros_msg_cb);
	ros::Publisher  test_msg_pub = nh.advertise<mission::Mavros_test_msg>("mavros/mavros_test_msg", 10);
	ros::Publisher  obs_pos_pub  = nh.advertise<mission::ObstaclePosition>("mavros/obstacle_position", 10);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    
    while(ros::ok()){

		mission::Mavros_test_msg msg_test;
		msg_test.test = 3;
		test_msg_pub.publish(msg_test);

		mission::ObstaclePosition obs_pos_test;
		obs_pos_test.obstacle_x = 3.0f;
		obs_pos_pub.publish(obs_pos_test);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}