#include <ur_kinematics/ur_kin.h>
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "robot.h"


Robot::Robot(ros::NodeHandle &nh) {
    n = nh;
    ik_client = n.serviceClient<ik_service::PoseIK>("pose_ik");
    trajectoryAction = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("/ariac/arm1/arm/follow_joint_trajectory/", true);
    jointStateSubscriber = n.subscribe("ariac/arm1/joint_states", 10, &Robot::jointStateCallback, this);
    if (!ros::service::waitForService("pose_ik", 10)) {
        ROS_ERROR("Did not find pose_ik service!");
    }
}

const sensor_msgs::JointState &Robot::getJointState() const {
    return jointState;
}

void Robot::jointStateCallback(const sensor_msgs::JointState_<std::allocator<void>>::ConstPtr &msg) {
    jointState = *msg;
}

void Robot::generateTrajectory(geometry_msgs::Point destination) {
    // create the matrices and vector for the kinematic math
    double T_pose[4][4], T_des[4][4];
    double q_pose[6], q_des[8][6];
    // do the forward kinematics to determine where the end effector is given the joint angles;
    for (int i = 0; i < 6; ++i) {
        q_pose[i] = jointState.position[i+1];
    }
    // run the FK solver
    ur_kinematics::forward((double *)&q_pose, (double *)&T_pose); // this is a bit of a wacky way of doing it.
    // the pose of the end effector relative to base_link
    T_des[0][3] = destination.x;
    T_des[1][3] = destination.y;
    T_des[2][3] = destination.z;
    T_des[3][3] = 1.0f;
    // The orientation of the end effector so that the end effector is down.
	T_des[0][0] =  0.0;  T_des[0][1] = -1.0; T_des[0][2] = 0.0;
	T_des[1][0] =  0.0;  T_des[1][1] =  0.0; T_des[1][2] = 1.0;
	T_des[2][0] = -1.0;  T_des[2][1] =  0.0; T_des[2][2] = 0.0;
	T_des[3][0] =  0.0;  T_des[3][1] =  0.0; T_des[3][2] = 0.0;
    // run the solver
    ur_kinematics::inverse((double *)&T_des, (double *)&q_des);
    // create the variable for the joint trajectory
    trajectory_msgs::JointTrajectory joint_trajectory;
    static int count = 0;
    // fill it out.
    joint_trajectory.header.seq = count++;
    joint_trajectory.header.stamp = ros::Time::now();
    joint_trajectory.header.frame_id = "/world";
    // set the names of the joints
    joint_trajectory.joint_names.clear();
    joint_trajectory.joint_names.emplace_back("linear_arm_actuator_joint");
    joint_trajectory.joint_names.emplace_back("shoulder_pan_joint");
    joint_trajectory.joint_names.emplace_back("shoulder_lift_joint");
    joint_trajectory.joint_names.emplace_back("elbow_joint");
    joint_trajectory.joint_names.emplace_back("wrist_1_joint");
    joint_trajectory.joint_names.emplace_back("wrist_2_joint");
    joint_trajectory.joint_names.emplace_back("wrist_3_joint");
    // set a start and end point
    joint_trajectory.points.resize(2);
    // set the start point to the current position of the joints from joint_states
    joint_trajectory.points[0].positions.resize(joint_trajectory.joint_names.size());
    for (int i_y = 0; i_y < joint_trajectory.joint_names.size(); ++i_y) {
        for (int i_z = 0; i_z < jointState.name.size(); ++i_z) {
            if (joint_trajectory.joint_names[i_y] == jointState.name[i_z]) {
                joint_trajectory.points[0].positions[i_y] = jointState.position[i_z];
                break;
            }
        }
    }
    // tell the arm when to start
    joint_trajectory.points[0].time_from_start = ros::Duration(0.0);
    // select which solution to use
    int q_des_idx = 0;
    joint_trajectory.points[1].positions.resize(joint_trajectory.joint_names.size());
    // set the linear_arm_actuator_joint from joint_states as it is not part of the inverse kinematics solution.
    joint_trajectory.points[1].positions[0] = jointState.position[1];
    // the actuators are commanded in an odd order, enter the joint positions in the correct positions
    for (int i_y = 0; i_y < 6; ++i_y) {
        joint_trajectory.points[1].positions[i_y + 1] = q_des[q_des_idx][i_y];
    }
    // tell the arm how long it has to perform the movement.
    joint_trajectory.points[1].time_from_start = ros::Duration(5.0);
}

void Robot::doTrajectory(trajectory_msgs::JointTrajectory &joint_trajectory) {
    control_msgs::FollowJointTrajectoryAction joint_trajectory_action;
    joint_trajectory_action.action_goal.goal.trajectory = joint_trajectory;
    actionlib::SimpleClientGoalState state = trajectoryAction->sendGoalAndWait(joint_trajectory_action.action_goal.goal, ros::Duration(30.0), ros::Duration(30.0));
    ROS_INFO("action server is in: [%i] %s", state.state_, state.toString().c_str());
}




