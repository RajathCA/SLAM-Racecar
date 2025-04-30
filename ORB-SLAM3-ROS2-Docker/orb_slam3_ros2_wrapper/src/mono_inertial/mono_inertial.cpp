#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "mono_inertial-slam-node.hpp"

int main(int argc, char **argv)
{
    if(argc < 3)
    {
        std::cerr << "\nUsage: ros2 run orb_slam3_ros2_wrapper mono_inertial path_to_vocabulary path_to_settings" << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);

    // auto node = std::make_shared<ORB_SLAM3_Wrapper::MonoInertialSlamNode>(argv[1], argv[2], ORB_SLAM3::System::IMU_MONOCULAR);
    auto node = std::make_shared<ORB_SLAM3_Wrapper::MonoInertialSlamNode>(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR);

    rclcpp::on_shutdown([node]() {
        RCLCPP_INFO(node->get_logger(), "Calling saveTrajectory on shutdown");
        node->saveTrajectories();  // You'll define this
    });
    
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(node);
    executor->spin();
    rclcpp::shutdown();

    return 0;
}

