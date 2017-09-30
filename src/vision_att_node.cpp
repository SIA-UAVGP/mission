/**
* @file  	vision_att_node.cpp
* @brief 	example: subscribe pixhawk attitude msg for vision using
* @author	huang li long <huanglilongwk@outlook.com>
*/

#include <ros/ros.h>
#include <mission/Attitude.h>
#include <geometry_msgs/TransformStamped.h>
ros::Publisher attitude_publisher;

mission::Attitude att_msg;
void att_cb(const mission::Attitude::ConstPtr& msg)
{
    att_msg = *msg;
    geometry_msgs::TransformStamped imu_att;
    imu_att.header.frame_id = "attitude";
    imu_att.header.stamp    = ros::Time::now();
    imu_att.transform.rotation.x = att_msg.roll;
    imu_att.transform.rotation.y = att_msg.pitch;
    imu_att.transform.rotation.z = att_msg.yaw;
    attitude_publisher.publish(imu_att);
    //printf("roll=%f; pit=%f; yaw=%f;\n", att_msg.roll, att_msg.pitch, att_msg.yaw);
    //ROS_INFO("yaw=%f;", att_msg.yaw);
	// TODO with att_msg
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision_att_node");
    ros::NodeHandle nh;

    ros::Subscriber att_sub = nh.subscribe<mission::Attitude>("mavros/attitude", 1, att_cb);
    attitude_publisher = nh.advertise<geometry_msgs::TransformStamped>("imu/attitude",1);

    ros::Rate rate(30.0);

    while(ros::ok())
    {
	
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
