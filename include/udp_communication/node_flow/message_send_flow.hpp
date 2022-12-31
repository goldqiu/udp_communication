
#ifndef UDP_COMMUNICATION_MESSAGE_SEND_FLOW_HPP_
#define UDP_COMMUNICATION_MESSAGE_SEND_FLOW_HPP_

#include <ros/ros.h>
// subscriber

#include "udp_communication/ros_interface/imu_subscriber.hpp"
#include "udp_communication/ros_interface/gnss_subscriber.hpp"
#include "udp_communication/global_defination/global_defination.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <string.h>
#include <arpa/inet.h>
#include <yaml-cpp/yaml.h>

namespace udp_communication {
class MessageSendFlow {
  public:
    MessageSendFlow(ros::NodeHandle& nh);
    bool Run();
    GNSSData current_gnss_data_;


  private:
    bool ReadData();
    bool HasData();
    bool ValidData();
    bool PublishData();
    bool InitSubscribers(ros::NodeHandle& nh, const YAML::Node& config_node);

  private:
    // subscriber
    std::shared_ptr<IMUSubscriber> imu_sub_ptr_;
    std::shared_ptr<GNSSSubscriber> gnss_sub_ptr_;


    std::deque<IMUData> imu_data_buff_;
    std::deque<GNSSData> gnss_data_buff_;

    IMUData current_imu_data_;

};
}

#endif