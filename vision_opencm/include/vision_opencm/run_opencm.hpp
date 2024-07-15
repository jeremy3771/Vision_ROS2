#ifndef RUN_OPENCM_HPP
#define RUN_OPENCM_HPP

#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#include <stdlib.h>
#include <stdio.h>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "vision_opencm/motor_driver.hpp"

#define PI                              3.141592653589793
#define AXLE_WIDTH                      0.2
#define WHEEL_BASE                      0.3

class DynamixelController : public rclcpp::Node, MotorCommand {
public:
    DynamixelController();
private:
    void twist_cb(const geometry_msgs::msg::Twist msg);
    void timer_cb();
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    int64_t motPos_[2];
};

#endif