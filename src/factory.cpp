#include "factory.h"

CompetitionOrders::CompetitionOrders(ros::NodeHandle &nh) {
    node = nh;

    materialLocationService = node.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");
    orderSubscriber = node.subscribe("/ariac/orders", 1000, &CompetitionOrders::orderSubscriberCallback, this);
    productTypeClient = node.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");
    orders.clear();
}

void CompetitionOrders::orderSubscriberCallback(const osrf_gear::Order::ConstPtr &msg) {
    orders.push_back(*msg);
}

void CompetitionOrders::processOrders() {
    osrf_gear::GetMaterialLocations matLoc;
    while (orders.empty()){
        ros::spinOnce();
    }
    while (!orders.empty()) { // check to see if there are any orders in the stack
        for (auto &shipment : orders[0].shipments){ // for each shipment in an order
            for (auto &product : shipment.products) { // for each product in a shipment
                matLoc.request.material_type = product.type; // formulate a request for where that product is
                if (productTypeClient.call(matLoc)) { // send it out
                    if(!matLoc.response.storage_units.empty()){
                        ROS_INFO("Object %s found in %s",
                                 matLoc.request.material_type.c_str(),
                                 matLoc.response.storage_units[0].unit_id.c_str());
                    } else {
                        ROS_WARN("did not find object %s in any storage unit.",
                                 matLoc.request.material_type.c_str());
                    }
                    locateProduct(product.type);
                } else {
                    ROS_ERROR("failed to call material_locations service!");
                }
            }
        }
        // pop the front-most order off of the stack
        orders.erase(orders.begin());
    }
}

void CompetitionOrders::locateProduct(std::string& productName) {
        materialLocations.request.material_type = productName;
        std::string identifiedBin;
        if (materialLocationService.call(materialLocations)) {
            identifiedBin = materialLocations.response.storage_units[0].unit_id;
            auto objects = imageMap["/ariac/logical_camera_" + identifiedBin];
            for (auto &model: objects->models) {
                if (model.type == productName){
                    ROS_INFO("located part %s in bin %s with pose w: %f x: %f y: %f z: %f x: %f y: %f z: %f",
                     productName.c_str(),
                     identifiedBin.c_str(),
                     model.pose.orientation.w,
                     model.pose.orientation.x,
                     model.pose.orientation.y,
                     model.pose.orientation.z,
                     model.pose.position.x,
                     model.pose.position.y,
                     model.pose.position.z
                     );
                }
            }

        } else {
            ROS_ERROR("failed to call material location service!");
        }
    }

void CompetitionOrders::subscriberCallback(const ros::MessageEvent<osrf_gear::LogicalCameraImage const>& event){
    const ros::M_string& header = event.getConnectionHeader();
    const std::string topic = header.at("topic");
    auto msg = event.getMessage();
    ROS_WARN("this is working, surprisingly enough.");
    imageMap[topic] = msg;
}