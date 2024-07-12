#include <chrono>
#include <functional>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "ddsm_ackermann/motor_command.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

#define PI 3.1415926535897932

class DDSM_Twist : public rclcpp::Node {
public:
    DDSM_Twist() : Node("DDSM115"), port1(1), port2(2), port3(3), port4(4){
        declare_parameter("WB", 0.3);
        declare_parameter("AW", 0.2);
        declare_parameter("WD", 0.1);
        get_parameter("WB", wheelbase_);
        get_parameter("AW", axleWidth_);
        get_parameter("WD", wheelDiameter_);

        twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 1, std::bind(&DDSM_Twist::twist_cb, this, _1));
        timer_ = this->create_wall_timer(50ms, std::bind(&DDSM_Twist::timer_cb, this));
    }
private:
    void timer_cb() {
        port1.SET_VELOCITY(1, motor_rpm_[0]);
        port2.SET_VELOCITY(1, motor_rpm_[1]);
        port3.SET_VELOCITY(1, motor_rpm_[2]);
        port4.SET_VELOCITY(1, motor_rpm_[3]);
    }

    void twist_cb(const geometry_msgs::msg::Twist msg) {
        double v = msg.linear.x;
        double omega = msg.angular.z;
        auto radius = v / omega;
        double cf = PI * wheelDiameter_;

        double omega1 = v * sqrt(pow((radius - axleWidth_ / 2), 2) + pow((wheelbase_ / 2), 2)) * radius / cf;
        double omega2 = v * sqrt(pow((radius + axleWidth_ / 2), 2) + pow((wheelbase_ / 2), 2)) * radius / cf;

        if (omega < 0) {
            motor_rpm_[0] = omega2 * 30 / PI;
            motor_rpm_[1] = -1 * omega1 * 30 / PI;
            motor_rpm_[2] = omega2 * 30 / PI;
            motor_rpm_[3] = -1 * omega1 * 30 / PI;
        }
        else if(omega > 0) {
            motor_rpm_[0] = omega1 * 30 / PI;
            motor_rpm_[1] = -1 * omega2 * 30 / PI;
            motor_rpm_[2] = omega1 * 30 / PI;
            motor_rpm_[3] = -1 * omega2 * 30 / PI;
        }
        else {
            motor_rpm_[0] = v / cf * 30 / PI;
            motor_rpm_[1] = v / cf * 30 / PI;
            motor_rpm_[2] = v / cf * 30 / PI;
            motor_rpm_[3] = v / cf * 30 / PI;
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    double motor_rpm_[4] = {0.0, }; // LF, RF, LR, RR
    MOTOR_COMMAND port1, port2, port3, port4;
    double wheelbase_, axleWidth_, wheelDiameter_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DDSM_Twist>());
    rclcpp::shutdown();
    return 0;
}
