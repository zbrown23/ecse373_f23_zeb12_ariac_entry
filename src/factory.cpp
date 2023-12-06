#include "factory.h"

CompetitionOrders::CompetitionOrders(ros::NodeHandle &nh) {
    for(auto &cameraName : cameraNames) {
        subscribers.push_back(node.subscribe(cameraName, 10, &CompetitionOrders::subscriberCallback, this));
    }
    orders.clear();
    node = nh;
    materialLocationService = node.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");
    orderSubscriber = node.subscribe("/ariac/orders", 1000, &CompetitionOrders::orderSubscriberCallback, this);
}

void CompetitionOrders::orderSubscriberCallback(const osrf_gear::Order::ConstPtr &msg) {
    orders.push_back(*msg);
    locateBin();
}

void CompetitionOrders::locateBin() {
    osrf_gear::GetMaterialLocations matLoc;
    auto product = orders[0].shipments[0].products[0];
    matLoc.request.material_type = product.type;
    if (materialLocationService.call(matLoc)) {
        if (!matLoc.response.storage_units.empty()) {
            ROS_INFO("Found %s in %s!", product.type.c_str(), matLoc.response.storage_units[0].unit_id.c_str());
            locateProduct(product.type, matLoc.response.storage_units);
        } else {
            ROS_WARN("material location service failed to find bins with %s!", product.type.c_str());
        }
    } else {
        ROS_ERROR("failed to call material location service!");
    }
}

void CompetitionOrders::locateProduct(std::string& productName, std::vector<osrf_gear::StorageUnit> &storage_units) {
    for (auto &storage_unit : storage_units) {
        auto it = cameraImages.find(storage_unit.unit_id);
        while (it == cameraImages.end()) {
            ros::spinOnce();
            it = cameraImages.find(storage_unit.unit_id);
        }
        if (!it->second.models.empty()) {
            for (auto &product: cameraImages[storage_unit.unit_id].models) {
                if (product.type == productName) {
                    printPose(product, storage_unit);
                    return;
                }
            }
        }
    }
}


static void printHelper(geometry_msgs::Pose& pose, std::string name) {
    ROS_WARN("%s pose:\n"
           "    position: \n\t x: %f \n\t y: %f \n\t z: %f \n"
           "    rotation: \n\t w: %f \n\t x: %f \n\t y: %f \n\t z: %f \n\n",
           name.c_str(),
           pose.position.x,
           pose.position.y,
           pose.position.z,
           pose.orientation.w,
           pose.orientation.x,
           pose.orientation.y,
           pose.orientation.z);
}

/**
 * Print the pose of the bin in the reference frame of both the camera and the base link of the robot
 */
void CompetitionOrders::printPose(osrf_gear::Model &product, osrf_gear::StorageUnit &storage_unit){
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped tfStamped;
    try {
        tfStamped = tfBuffer.lookupTransform("arm1_base_link", "logical_camera_" + storage_unit.unit_id + "_frame",
                                             ros::Time(0.0), ros::Duration(1.0));
    } catch (tf2::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
    }
    geometry_msgs::PoseStamped part_pose, goal_pose;
    part_pose.pose = product.pose;
    printHelper(part_pose.pose, "logical_camera_" + storage_unit.unit_id + "_frame");
    tf2::doTransform(part_pose, goal_pose, tfStamped);
    printHelper(goal_pose.pose, "arm1_base_link");
}


void CompetitionOrders::subscriberCallback(const ros::MessageEvent<osrf_gear::LogicalCameraImage const>& event) {
    const ros::M_string& header = event.getConnectionHeader();
    const std::string topic = header.at("topic");
    auto msg = event.getMessage();
    auto map_name = topic.substr(22);
    cameraImages[map_name] = *msg;
}