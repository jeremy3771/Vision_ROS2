#include "vision_opencm/run_opencm.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

DynamixelController::DynamixelController() : Node("dynamixel_controller") {
    sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&DynamixelController::twist_cb ,this, _1));
    timer_ = this->create_wall_timer(10ms, std::bind(&DynamixelController::timer_cb, this));
}
void DynamixelController::twist_cb(const geometry_msgs::msg::Twist msg) {
    double radius = std::abs(msg.linear.x / msg.angular.z);
    if (msg.angular.z < -0.01) {
        motPos_[0] = 2048 + (std::atan2(WHEEL_BASE, radius + (AXLE_WIDTH / 2)) * 2048 / PI);
        motPos_[1] = 2048 + (std::atan2(WHEEL_BASE, radius - (AXLE_WIDTH / 2)) * 2048 / PI);
    }
    else if (msg.angular.z > 0.01) {
        motPos_[0] = 2048 - (std::atan2(WHEEL_BASE, radius - (AXLE_WIDTH / 2)) * 2048 / PI);
        motPos_[1] = 2048 - (std::atan2(WHEEL_BASE, radius + (AXLE_WIDTH / 2)) * 2048 / PI);
    }
}
void DynamixelController::timer_cb() {
    writePosition(motPos_[0], motPos_[1]);
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DynamixelController>());
    rclcpp::shutdown();
    return 0;
}