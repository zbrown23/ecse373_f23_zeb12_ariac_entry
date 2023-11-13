#pragma once

#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "std_srvs/SetBool.h"

// class encapsulating the ARIAC competition data.
class Competition {
private:
    ros::NodeHandle node;
    std_srvs::Trigger begin_comp;
    ros::ServiceClient begin_client;
public:
    Competition(ros::NodeHandle& nh);
    void begin_competition();
};