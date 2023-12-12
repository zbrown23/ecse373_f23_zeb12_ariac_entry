#include "ros/ros.h"
#include <sstream>
#include "competition.h"
#include "factory.h"
#include "robot.h"

// update code to use ik service
// for all services, add a call to ros::serivce::waitForService(<service_name>, [timeout]);

int main(int argc, char **argv) {
    ros::init(argc, argv, "ariac_entry");
    ros::NodeHandle n;
    Competition competition(n);
    CompetitionOrders orders(n);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    competition.begin_competition();
    while (true) {
        // use ROS_INFO_THROTTLE to publish the current joint states at the time of the most recent message every 10 seconds.
    }
    return 0;
}