#include "ros/ros.h"
#include <sstream>
#include "competition.h"
#include "factory.h"



int main(int argc, char **argv) {
    ros::init(argc, argv, "ariac_entry");
    ros::NodeHandle n;
    Competition competition(n);
    CompetitionOrders orders(n);
    competition.begin_competition();
    orders.processOrders();
    ros::spin();
    return 0;
}