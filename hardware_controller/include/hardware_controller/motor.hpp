#pragma once

#include <can_msgs/msg/motor_msg.hpp>
#include <can_msgs/srv/set_pidf_gains.hpp>
#include <string>
#include <rclcpp/rclcpp.hpp>

namespace motors
{
    struct JointState {
        std::string name;
        double position, velocity, effort;
    };

    class Motor{
        public:

        virtual void declareConfig(std::shared_ptr<rclcpp::Node>) = 0;

        virtual void executeConfig(std::shared_ptr<rclcpp::Node>) = 0;

        virtual void setValue(std::shared_ptr<can_msgs::msg::MotorMsg> msg) = 0;

        virtual void configMotorPIDF(const std::shared_ptr<can_msgs::srv::SetPIDFGains::Request> req,
                         std::shared_ptr<can_msgs::srv::SetPIDFGains::Response> resp) = 0;

        virtual JointState getJointState() = 0;

    };

    struct MotorContainer {
        Motor& motor;
        rclcpp::Subscription<can_msgs::msg::MotorMsg>::SharedPtr sub;
        rclcpp::Service<can_msgs::srv::SetPIDFGains>::SharedPtr pidfSrv;
    };
} // namespace motors
