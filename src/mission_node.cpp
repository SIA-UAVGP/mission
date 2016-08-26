/**
* @file     : mission_node.cpp
* @brief    : take off --> pos A --> pos B --> land
* @author   : huang li long <huanglilongwk@outlook.com>
* @time     : 2016/07/27
*/

#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mission/CommandBool.h>
#include <mission/SetMode.h>
#include <mission/State.h>
#include <mission/Attitude.h>

#include <mission/CommandTOL.h>

//#include <mavros/frame_tf.h>

void state_machine(void);

// state machine's states
static const int POS_A = 0;
static const int POS_B = 1;
static const int LAND  = 2;

// set A, B and land position setpoint for publish
geometry_msgs::PoseStamped pose_a;
geometry_msgs::PoseStamped pose_b;

// pulisher
ros::Publisher local_pos_pub;

// current positon state, init state is takeoff
int current_pos_state = POS_A;

// state msg callback function
mission::State current_state;
void state_cb(const mission::State::ConstPtr& msg){
    current_state = *msg;
}

// local position msg callback function
geometry_msgs::PoseStamped current_pos;
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pos = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mission_node");
    ros::NodeHandle nh;

    // get pixhawk's status
    ros::Subscriber state_sub = nh.subscribe<mission::State>("mavros/state", 10, state_cb);

    // get pixhawk's local position
    ros::Subscriber pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, pos_cb);       

    // publisher postion setpoint to pixhawk
    // ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

    // land service 
    ros::ServiceClient land_client = nh.serviceClient<mission::CommandTOL>("mavros/cmd/land");

    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    // position setpoint A
    pose_a.pose.position.x = 0;
    pose_a.pose.position.y = 0;
    pose_a.pose.position.z = 5;

    // position setpoint B
    pose_b.pose.position.x = 0;
    pose_b.pose.position.y = 5;
    pose_b.pose.position.z = 5;

	// auto quat_yaw = mavros::ftf::quaternion_from_rpy(0.0, 0.0, 0.0);
	// pose_a.pose.orientation.x = quat_yaw.x();
	// pose_a.pose.orientation.y = quat_yaw.y();
	// pose_a.pose.orientation.z = quat_yaw.z();
	// pose_a.pose.orientation.w = quat_yaw.w();
	
	// pose_b.pose.orientation.x = quat_yaw.x();
	// pose_b.pose.orientation.y = quat_yaw.y();
	// pose_b.pose.orientation.z = quat_yaw.z();
	// pose_b.pose.orientation.w = quat_yaw.w();

    //send a few setpoints before starting  --> for safety
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose_a);
        ros::spinOnce();
        rate.sleep();
    }

    mission::CommandTOL landing_cmd;
    landing_cmd.request.min_pitch = 1.0;

    ros::Time landing_last_request = ros::Time::now();

    while(ros::ok()){

		if(!current_state.armed)
		{
			current_pos_state = POS_A;
		}

        // auto task off
        if( current_state.mode == "AUTO.TAKEOFF"){
            ROS_INFO("AUTO TAKEOFF!");
        }

        // landing
        if((current_pos_state == LAND) && (current_state.mode != "OFFBOARD")){                  
            
            if( current_state.mode != "AUTO.LAND" &&
            (ros::Time::now() - landing_last_request > ros::Duration(5.0))){
            if(land_client.call(landing_cmd) &&
                landing_cmd.response.success){
                ROS_INFO("AUTO LANDING!");
            }
            landing_last_request = ros::Time::now();
            }
        }

        state_machine();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

// task state machine
void state_machine(void){

    switch(current_pos_state){
        
        case POS_A:
            local_pos_pub.publish(pose_a);      // first publish pos A 
            if((abs(current_pos.pose.position.x - pose_a.pose.position.x) < 0.1) &&      // switch to next state
               (abs(current_pos.pose.position.y - pose_a.pose.position.y) < 0.1) &&
               (abs(current_pos.pose.position.z - pose_a.pose.position.z) < 0.1))
               {
                    current_pos_state = POS_B; // current_pos_state++;
               }
            break;
        case POS_B:
            local_pos_pub.publish(pose_b);      // first publish pos B 
            if((abs(current_pos.pose.position.x - pose_b.pose.position.x) < 0.1) &&      // switch to next state
               (abs(current_pos.pose.position.y - pose_b.pose.position.y) < 0.1) &&
               (abs(current_pos.pose.position.z - pose_b.pose.position.z) < 0.1))
               {
                    current_pos_state = LAND; // current_pos_state++;
               }
       	    break;
        case LAND:
            break;
    }
}