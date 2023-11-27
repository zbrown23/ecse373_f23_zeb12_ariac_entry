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
    ros::NodeHandle node;
    osrf_gear::GetMaterialLocations materialLocations;
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
    std::vector<ros::Subscriber> subscribers;
    std::vector<boost::function<void(const osrf_gear::LogicalCameraImage::ConstPtr&)>> callbacks;
public:
    Cameras(ros::NodeHandle& nh) {
        for (const auto& cameraName : cameraNames) {
            callbacks.emplace_back(boost::bind(&Cameras::subscriberCallback, this, _1, cameraName));
            subscribers.push_back(nh.subscribe(cameraName, 1, callbacks.back()));
        }
        materialLocationService = node.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");
    }
    void locateProduct(std::string& productName) {
        materialLocations.request.material_type = productName;
        std::string identifiedBin;
        if (materialLocationService.call(materialLocations)) {
            identifiedBin = materialLocations.response.storage_units[0].unit_id;

        } else {
            ROS_ERROR("failed to call material location service!");
        }


    // search through camera data for the bin identified by material_location, and spit out location with ROS_INFO_STREAM
    }
    void subscriberCallback(const osrf_gear::LogicalCameraImage::ConstPtr& msg, const std::string& cameraName) {
        // store locations of components in camera view
        ROS_INFO("Received message from %s", cameraName.c_str());
    }


};


