/**
* @file     : mission.h
* @brief    : state machine class
* @author   : huang li long <huanglilongwk@outlook.com>
* @time     : 2016/08/26
*/

#pragma once

#include <ros/ros.h>
#include <mission/quat_euler.h>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mission/State.h>
#include <mission/CommandTOL.h>
#include <dynamic_reconfigure/server.h>
#include <mission/missionConfig.h>

namespace mission{

/*
 * @brief  main state
 *
 */
enum MAIN_STATE
{
	M_IDLE =0,
	M_DISPLAY,
	M_DIGITAL_0,
	M_DIGITAL_1,
	M_DIGITAL_2,
	M_DIGITAL_3,
	M_DIGITAL_4,
	M_DIGITAL_5,
	M_DIGITAL_6,
	M_DIGITAL_7,
	M_DIGITAL_8,
	M_DIGITAL_9,
	M_LAND
};

/*
 * @brief	paramters structure
 *
 */
struct params_s
{
	double local_alt_sp;
	double yaw_sp;
};

/*
 * @brief  state machine class
 *
 */
 class Mission
 {
	 public:
	 	Mission();
		~Mission();

		void mission_main(void);							// entry point

		geometry_msgs::PoseStamped _current_local_pos;		// uav current local postion
		mission::State 			   _current_state;			// uav current arm status and mode

		geometry_msgs::PoseStamped _local_pos_sp; 			// actual local pos sent to uav

		uint8_t _main_state;								// main state for state machine
		ros::NodeHandle _node_handle;
		ros::Rate _rate;
	 private:
		 // state machine
		 void state_machine(void);

		 // callback functions
		 void state_cb(const mission::State::ConstPtr& msg);
		 void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);

		 void wait_connect(void);

		 void cmd_streams(void);				// pixhawk need cmd streams, before switch to OFFBOARD mode

		 void set_yaw_sp(geometry_msgs::PoseStamped &pose, const double yaw);		// set yaw setpoint
		 void set_pos_sp(geometry_msgs::PoseStamped &pose, const double x, const double y, const double z);	// set position setpoint

		 // dynamic reconfigure
		 void params_cfg_cb(missionConfig &config);

	 	 // Subscribers 
		 ros::Subscriber _state_sub;			// get pixhawk's arming and status
		 ros::Subscriber _cur_pos_sub;			// get pixhawk current local position

		 // Publishers
		 ros::Publisher _local_pos_sp_pub;		// pusblish local position setpoint to pixhawk

		 // Services
		 ros::ServiceClient _land_client;		// land command

		 // dynamic reconfigure
		 struct params_s _params;
		 dynamic_reconfigure::Server<missionConfig> _param_server;
		 dynamic_reconfigure::Server<missionConfig>::CallbackType _param_cfg;

		 mission::CommandTOL _landing_cmd;
		 ros::Time _landing_last_request;
	};

}	// end of mission namespace
