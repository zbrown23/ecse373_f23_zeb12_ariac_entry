#pragma once

#include "ros/ros.h"
#include <vector>
#include <osrf_gear/GetMaterialLocations.h>
#include <osrf_gear/Order.h>
#include <osrf_gear/StorageUnit.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/GetMaterialLocations.h>
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
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
    void locateBin();
    void printPose(osrf_gear::Model &product, osrf_gear::StorageUnit &storage_unit);
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
    std::map<std::string, osrf_gear::LogicalCameraImage> cameraImages;
    std::vector<ros::Subscriber> subscribers;
public:
    void locateProduct(std::string& productName, std::vector<osrf_gear::StorageUnit> &storage_units);
    void subscriberCallback(const ros::MessageEvent<osrf_gear::LogicalCameraImage const>& event);
};


