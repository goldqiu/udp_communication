#include "udp_communication/node_flow/message_send_flow.hpp"

#include "glog/logging.h"
#include "udp_communication/global_defination/global_defination.h"
#include <vector>

namespace udp_communication {
MessageSendFlow::MessageSendFlow(ros::NodeHandle& nh) {
    std::string config_file_path = WORK_SPACE_PATH + "/config/config.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);

    InitSubscribers(nh, config_node["measurements"]);
}

bool MessageSendFlow::InitSubscribers(ros::NodeHandle& nh, const YAML::Node& config_node) {

    //
    // init input message subs:
    // 
    imu_sub_ptr_ = std::make_shared<IMUSubscriber>(
        nh, 
        config_node["imu"]["topic_name"].as<std::string>(), config_node["imu"]["queue_size"].as<int>()
    );
    gnss_sub_ptr_ = std::make_shared<GNSSSubscriber>(
        nh, 
        config_node["gnss"]["topic_name"].as<std::string>(), config_node["gnss"]["queue_size"].as<int>()
    );
    return true;
}

bool MessageSendFlow::Run() {

    if (!ReadData())
        return false;

    while(HasData()) {
        if (!ValidData())
            continue;

        PublishData();
    }

    return true;
}

bool MessageSendFlow::ReadData() {

    imu_sub_ptr_->ParseData(imu_data_buff_);
    gnss_sub_ptr_->ParseData(gnss_data_buff_);

    if (gnss_data_buff_.size() == 0)
    return false;

    return true;
}



bool MessageSendFlow::HasData() {

    if (imu_data_buff_.size() == 0)
        return false;
    if (gnss_data_buff_.size() == 0)
        return false;

    return true;
}

bool MessageSendFlow::ValidData() {

    current_imu_data_ = imu_data_buff_.front();
    current_gnss_data_ = gnss_data_buff_.front();

    imu_data_buff_.clear();
    gnss_data_buff_.clear();

    return true;
}


bool MessageSendFlow::PublishData() {

    std::cout<<current_gnss_data_.time<<std::endl;
    return true;
}

}