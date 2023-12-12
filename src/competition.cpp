#include "competition.h"


Competition::Competition(ros::NodeHandle &nh) {
    node = nh;
    begin_client = node.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
    if (!ros::service::waitForService("/ariac/start_competition", 5)) {
        ROS_ERROR("Could not find start_competition service!");
    }
}

void Competition::begin_competition() {
    if (begin_client.call(begin_comp)) {
        if(begin_comp.response.success){
            ROS_INFO("Competition service called successfully: %s",
                     begin_comp.response.message.c_str());
        } else {
            ROS_WARN("Competition service returned failure: %s",
                     begin_comp.response.message.c_str());
        }
    } else {
        ROS_ERROR("failed to call competition service!");
    }
}