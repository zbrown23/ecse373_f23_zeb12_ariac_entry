#pragma once
#include <ros/service_client.h>
#include <ros/node_handle.h>
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "ik_service/PoseIK.h"
#include "ros/service.h"

class Robot {
private:
    ros::NodeHandle n;
    ros::Subscriber jointStateSubscriber;
    sensor_msgs::JointState jointState;
    ros::ServiceClient ik_client;
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> *trajectoryAction;
    void doTrajectory(trajectory_msgs::JointTrajectory &joint_trajectory);
public:
    Robot(ros::NodeHandle &nh);
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg);
    void generateTrajectory(geometry_msgs::Point destination);
    const sensor_msgs::JointState &getJointState() const;
};