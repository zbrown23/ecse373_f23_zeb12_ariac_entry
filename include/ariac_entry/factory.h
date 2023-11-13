#pragma once

#include "ros/ros.h"
#include <vector>
#include <osrf_gear/GetMaterialLocations.h>
#include <osrf_gear/Order.h>
#include <osrf_gear/StorageUnit.h>
#include<iterator>

class CompetitionOrders {
private:
    ros::NodeHandle node;
    ros::Subscriber orderSubscriber;
    std::vector<osrf_gear::Order> orders;
    ros::ServiceClient productTypeClient;
public:
    CompetitionOrders(ros::NodeHandle& nh);
    void orderSubscriberCallback(const osrf_gear::Order::ConstPtr& msg);
    void processOrders();
};

class Cameras {
private:
    ros::ServiceClient materialLocationClient;
public:
};