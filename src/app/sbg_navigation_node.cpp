#include <cstdlib>
#include <ros/ros.h>
#include <pcl/common/transforms.h>

#include "udp_communication/global_defination/global_defination.h"
#include "glog/logging.h"


//
// subscribers:
//
#include "udp_communication/ros_interface/imu_subscriber.hpp"
#include "udp_communication/ros_interface/gnss_subscriber.hpp"
//
// publishers:
//
#include "udp_communication/ros_interface/odometry_publisher.hpp"

using namespace udp_communication;

void GetTransformIMUToMap(
    GNSSData &gnss_data, IMUData &imu_data,
    Eigen::Matrix4f &imu_to_map
) {
    //
    // init
    // 
    gnss_data.UpdateXYZ();
    //
    // a. set position:
    // 
    imu_to_map(0,3) = gnss_data.local_E;
    imu_to_map(1,3) = gnss_data.local_N;
    imu_to_map(2,3) = gnss_data.local_U;
    //
    // b. set orientation:
    //
    imu_to_map.block<3,3>(0,0) = imu_data.GetOrientationMatrix();
}


int main(
    int argc, 
    char *argv[]
) {
    google::InitGoogleLogging(argv[0]);
    
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "sbg_navigation_node");
    ros::NodeHandle nh;

    std::shared_ptr<IMUSubscriber> imu_sub_ptr = std::make_shared<IMUSubscriber>(nh, "/imu_raw", 1000000);
    std::shared_ptr<GNSSSubscriber> gnss_sub_ptr = std::make_shared<GNSSSubscriber>(nh, "/imu/nav_sat_fix", 1000000);  
    //适配实验室小车话题

    //
    // register publishers:
    //
    std::shared_ptr<OdometryPublisher> odom_pub_ptr = std::make_shared<OdometryPublisher>(nh, "lidar_odom", "/map", "velo_link", 100);

    std::deque<IMUData> imu_data_buff;
    std::deque<GNSSData> gnss_data_buff;

    Eigen::Matrix4f imu_to_map = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f lidar_to_imu = Eigen::Matrix4f::Identity();
   
    bool transform_received = false;
    bool gnss_origin_position_inited = false;
    lidar_to_imu.block<3,3>(0,0) << 0 , -1 , 0 , 1 , 0 , 0 , 0 , 0, 1;

    ros::Rate rate(100);

    while (ros::ok()) {
        ros::spinOnce();

        imu_sub_ptr->ParseData(imu_data_buff);
        gnss_sub_ptr->ParseData(gnss_data_buff);
     
        while (
                !imu_data_buff.empty() && 
                !gnss_data_buff.empty()
        ) {
            IMUData imu_data = imu_data_buff.front();
            GNSSData gnss_data = gnss_data_buff.front();

            if (!gnss_origin_position_inited) {
                        gnss_data.InitOriginPosition();
                        gnss_origin_position_inited = true;
            }
            GetTransformIMUToMap(
                        gnss_data, imu_data,
                        imu_to_map
            );
            Eigen::Matrix4f lidar_odometry = imu_to_map * lidar_to_imu;
   
            odom_pub_ptr->Publish(lidar_odometry);

            imu_data_buff.clear();
            gnss_data_buff.clear();
        }
        rate.sleep();
    }
    return EXIT_SUCCESS;
}