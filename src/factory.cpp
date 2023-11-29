#include "factory.h"

void CompetitionOrders::waitUntilFullMap(){
    int count = 0;
    while(count != 10){
        ros::spinOnce();
        for(const auto& key : cameraNames) {
         if (imageMap.find(key) != imageMap.end()) {
             count++;
         }
        }
    }
}

CompetitionOrders::CompetitionOrders(ros::NodeHandle &nh) {
    node = nh;
    materialLocationService = node.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");
    orderSubscriber = node.subscribe("/ariac/orders", 1000, &CompetitionOrders::orderSubscriberCallback, this);
    for(auto &cameraName : cameraNames) {
        subscribers.push_back(node.subscribe(cameraName, 10, &CompetitionOrders::subscriberCallback, this));
    }
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
                if (materialLocationService.call(matLoc)) { // send it out
                    if(!matLoc.response.storage_units.empty()){
                        ROS_INFO("%s found in %s",
                                 matLoc.request.material_type.c_str(),
                                 matLoc.response.storage_units[0].unit_id.c_str());
                        locateProduct(product.type);
                    } else {
                        ROS_WARN("did not find %s in any storage unit.",
                                 matLoc.request.material_type.c_str());
                    }
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
            try {
                int binNum = 0;
                do {
                    binNum++;
                    identifiedBin = materialLocations.response.storage_units[binNum].unit_id;
                    ROS_INFO("looking for %s in /ariac/logical_camera_%s", productName.c_str(), identifiedBin.c_str());
                } while (imageMap.find("/ariac/logical_camera_" + identifiedBin) != imageMap.end());
                    auto objects = imageMap.at("/ariac/logical_camera_" + identifiedBin);
                    for (auto &model: objects.models) {
                        ROS_INFO("checking %s against %s", model.type.c_str(), productName.c_str());
                        if (model.type == productName) {
                            ROS_INFO("using cameras located part %s in bin %s at w:%f x:%f y:%f z:%f x:%f y:%f z:%f",
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
                        } else {
                            ROS_WARN("failed to locate %s in bin %s!", productName.c_str(), identifiedBin.c_str());
                        }
                    }
            } catch (const std::exception &e) {
                    ROS_ERROR("failed to locate camera /ariac/logical_camera_%s in map!", identifiedBin.c_str());
            }
        } else {
            ROS_ERROR("failed to call material location service!");
        }
}

void CompetitionOrders::subscriberCallback(const ros::MessageEvent<osrf_gear::LogicalCameraImage const>& event){
    const ros::M_string& header = event.getConnectionHeader();
    const std::string topic = header.at("topic");
    auto msg = event.getMessage();
    imageMap[topic] = *msg;
}