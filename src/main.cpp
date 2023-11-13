#include "ros/ros.h"
#include <sstream>
#include "competition.h"
#include "factory.h"



int main(int argc, char **argv)
{
    ros::init(argc, argv, "ariac_entry");
    ros::NodeHandle n;
    Competition Competition(n);
    CompetitionOrders orders(n);


  return 0;
}