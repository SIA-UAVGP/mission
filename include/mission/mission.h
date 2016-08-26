/**
* @file     : mission.h
* @brief    : state machine class
* @author   : huang li long <huanglilongwk@outlook.com>
* @time     : 2016/08/26
*/

#pragma once

#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>

namespace mission{

/*
 * @brief  state machine class
 *
 */
 class Mission
 {
	 public:
	 	Mission();
		~Mission(){};
	 
	 private:
	 	 // Subscribers 
		 ros::Subscriber _state_sub;			// get pixhawk's arming and status
		 ros::Subscriber _cur_pos_sub;			// get pixhawk current local position

		 // Publishers
		 ros::Publisher _local_pos_pub;			// pusblish local position setpoint to pixhawk

		 // Services
		 ros::ServiceClient _land_client;		// land command
 }


}	// end of mission namespace