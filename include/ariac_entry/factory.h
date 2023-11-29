#pragma once

#include "ros/ros.h"
#include <vector>
#include <osrf_gear/GetMaterialLocations.h>
#include <osrf_gear/Order.h>
#include <osrf_gear/StorageUnit.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/GetMaterialLocations.h>
#include<iterator>

class CompetitionOrders {
private:
    osrf_gear::GetMaterialLocations materialLocations;
    ros::NodeHandle node;
    ros::Subscriber orderSubscriber;
    std::vector<osrf_gear::Order> orders;
    ros::ServiceClient productTypeClient;
public:
    CompetitionOrders(ros::NodeHandle& nh);
    void orderSubscriberCallback(const osrf_gear::Order::ConstPtr& msg);
    void processOrders();
private:
    ros::ServiceClient materialLocationService;
    std::vector<std::string> cameraNames = {
        "/ariac/logical_camera_agv1",
        "/ariac/logical_camera_agv2",
        "/ariac/logical_camera_bin1",
        "/ariac/logical_camera_bin2",
        "/ariac/logical_camera_bin3",
        "/ariac/logical_camera_bin4",
        "/ariac/logical_camera_bin5",
        "/ariac/logical_camera_bin6",
        "/ariac/quality_control_sensor_1",
        "/ariac/quality_control_sensor_2"
    };
    std::map<std::string, osrf_gear::LogicalCameraImage> imageMap;
    std::vector<ros::Subscriber> subscribers;
    void waitUntilFullMap();
public:
    void locateProduct(std::string& productName);
    void subscriberCallback(const ros::MessageEvent<osrf_gear::LogicalCameraImage const>& event);
};


