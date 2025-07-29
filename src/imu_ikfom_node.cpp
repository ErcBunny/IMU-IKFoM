#include "imu_ikfom/imu_ikfom_node.hpp"

int main(const int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImuIkfomNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
