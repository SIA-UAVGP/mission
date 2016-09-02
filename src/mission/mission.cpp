/**
* @file     : mission.cpp
* @brief    : state machine class
* @author   : huang li long <huanglilongwk@outlook.com>
* @time     : 2016/08/26
*/

#include <mission/mission.h>

using namespace mission;

// constructor
Mission::Mission():
_main_state(M_IDLE),
_rate(20.0)
{
    // clear all members's value here

}

// deconstructor
Mission::~Mission()
{

}

// public interface
void
Mission::mission_main(void)
{
    // subscribers init
    _state_sub = _node_handle.subscribe("mavros/state", 10, &Mission::state_cb, this);
    _cur_pos_sub = _node_handle.subscribe("mavros/local_position/pose", 10, &Mission::local_pos_cb, this);

    // publishers init
    _local_pos_sp_pub = _node_handle.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

    // services init
    _land_client = _node_handle.serviceClient<mission::CommandTOL>("mavros/cmd/land");

	// dynamic reconfigure service init
	_param_cfg = boost::bind(&Mission::params_cfg_cb, this, _1);
	_param_server.setCallback(_param_cfg);

    // wait for FCU connection
    wait_connect();

    //send a few setpoints before switch to OFFBOARD mode
    cmd_streams();

    // for landing service
    _landing_last_request = ros::Time::now();

    // state machine
    while(ros::ok()){

        // for debug convenience
		if(!_current_state.armed)
		{
			_main_state = M_IDLE;
		}

        state_machine();
        ros::spinOnce();
        _rate.sleep();
    }
}

void
Mission::state_machine(void)
{
    /***************** for test ***********************/
    geometry_msgs::PoseStamped pose_a;
    geometry_msgs::PoseStamped pose_b;

	set_pos_sp(pose_a, 0.0, 0.0, 3.0);
	set_yaw_sp(pose_a, 3.14);

	set_pos_sp(pose_b, 0.0, 3.0, 3.0);
	set_yaw_sp(pose_b, 3.14 /2);
    /***************** for test ***********************/

    switch(_main_state){
        case M_IDLE:
            _local_pos_sp_pub.publish(_current_local_pos);
            if(_current_state.armed && _current_state.mode == "OFFBOARD")    // start mission
            {
                _main_state = M_DISPLAY;
            }
            break;
        case M_DISPLAY:
            _local_pos_sp_pub.publish(pose_a);      // publish display's local position
            if((abs(_current_local_pos.pose.position.x - pose_a.pose.position.x) < 0.1) &&
               (abs(_current_local_pos.pose.position.y - pose_a.pose.position.y) < 0.1) &&
               (abs(_current_local_pos.pose.position.z - pose_a.pose.position.z) < 0.1))
               {
                    _main_state = M_DIGITAL_0; // get digital number from display
               }
            break;
        case M_DIGITAL_0:
            _local_pos_sp_pub.publish(pose_b);      // start sub state machine, here just publish local pos for debug convenience
            if((abs(_current_local_pos.pose.position.x - pose_b.pose.position.x) < 0.1) &&
               (abs(_current_local_pos.pose.position.y - pose_b.pose.position.y) < 0.1) &&
               (abs(_current_local_pos.pose.position.z - pose_b.pose.position.z) < 0.1))
               {
                    _main_state = M_LAND;
               }
            break;
        case M_LAND:
            if(_current_state.mode == "OFFBOARD"){

                if(_current_state.mode != "AUTO.LAND" &&
                (ros::Time::now() - _landing_last_request > ros::Duration(5.0))){
                if(_land_client.call(_landing_cmd) &&
                    _landing_cmd.response.success){
                    ROS_INFO("AUTO LANDING!");
                }
                _landing_last_request = ros::Time::now();
                }
            }
            break;
    }
}

// uav state subscriber's callback function
void
Mission::state_cb(const mission::State::ConstPtr& msg)
{
    _current_state = *msg;
}

// local pos subscriber's callback function
void
Mission::local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    _current_local_pos = *msg;
}

// wait for mavros connecting with pixhawk
void
Mission::wait_connect(void)
{
    while(ros::ok() && !_current_state.connected){
        ros::spinOnce();
        _rate.sleep();
    }
}

// send commands streams to pixhawk before switch to OFFBOARD mode
void
Mission::cmd_streams(void)
{
    // publish current local position
    for(int i = 100; ros::ok() && i > 0; --i){
        _local_pos_sp_pub.publish(_current_local_pos);
        ros::spinOnce();
        _rate.sleep();
    }
}

// set yaw setpoint -- unit: rad
void
Mission::set_yaw_sp(geometry_msgs::PoseStamped &pose, const double yaw)
{
	Eigen::Quaterniond quat_yaw =  mission::tf::quaternion_from_rpy(0.0, 0.0, yaw);
	pose.pose.orientation.x = quat_yaw.x();
	pose.pose.orientation.y = quat_yaw.y();
	pose.pose.orientation.z = quat_yaw.z();
	pose.pose.orientation.w = quat_yaw.w();
}

// set position setpoint -- unit: m
void
Mission::set_pos_sp(geometry_msgs::PoseStamped &pose, const double x, const double y, const double z)
{
	pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;
}

// dynamic reconfigure
void
Mission::params_cfg_cb(missionConfig &config)
{
	// display reconfigure msgs
	ROS_INFO("local pos sp: %f", config.local_pos_sp);
}
