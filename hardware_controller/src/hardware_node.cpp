#include <can_msgs/msg/motor_msg.hpp>
#include <can_msgs/srv/set_pidf_gains.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>

#define Phoenix_No_WPI  // remove WPI dependencies
#include <algorithm>
#include <cctype>
#include <exception>
#include <memory>
#include <string>

#include "ctre/Phoenix.h"
#include "ctre/phoenix/cci/Unmanaged_CCI.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "hardware_controller/motor.hpp"
#include "hardware_controller/talon_brushless.hpp"

#define SAFETY_TIMEOUT_MS 100
using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

class HardwareNode : public rclcpp::Node {
 public:
    HardwareNode() : Node("hardware_node") {
        RCLCPP_DEBUG(get_logger(), "Initializing safety subscriber");

        safetySubscrip = create_subscription<std_msgs::msg::Bool>(
            "safety_enable", rclcpp::SystemDefaultsQoS(),
            std::bind(&HardwareNode::feedSafety, this, _1));

        RCLCPP_DEBUG(get_logger(), "Initializing data publisher");
        motorStatePub = create_publisher<sensor_msgs::msg::JointState>(
            "motor/joint_state", rclcpp::SensorDataQoS());

        RCLCPP_DEBUG(get_logger(), "Initializing timers");

        highRate = create_wall_timer(
            10ms, std::bind(&HardwareNode::highRateCallback, this));
        highRate->cancel();

        RCLCPP_INFO(get_logger(), "Hardware controller initalized");
    }

    void createMotors() {
        // create the diagnostic updater
        updater = std::make_shared<diagnostic_updater::Updater>(this->shared_from_this());
        updater->setHardwareID("none");
        updater->add("Can Bus Stats", this, &HardwareNode::can_diagnostics);
        updater->force_update();

        std::vector<std::string> motorNames;
        RCLCPP_DEBUG(get_logger(), "Loading motors from config");
        try {
            this->declare_parameter<std::vector<std::string>>("motor_names", {});
            motorNames = this->get_parameter("motor_names").as_string_array();
        } catch (std::runtime_error) {
            // This is what happens if the array is empty
            RCLCPP_FATAL(get_logger(), "No motors declared in config!");
        }

        for (auto motorName : motorNames) {
            RCLCPP_DEBUG_STREAM(get_logger(), "Building motor " << motorName);
            this->declare_parameter<int>(motorName + ".id", 0);
            this->declare_parameter<std::string>(motorName + ".type", "");

            auto type = this->get_parameter(motorName + ".type").as_string();
            std::transform(type.begin(), type.end(), type.begin(),
                           [](unsigned char c) { return std::tolower(c); });

            motors::Motor *motorBase;
            if (type.find("talonfx") != std::string::npos) {
                motorBase = new motors::TalonBrushless(
                    this->get_parameter(motorName + ".id").as_int(),
                    motorName);
            } else {
                RCLCPP_ERROR_STREAM(get_logger(), "Error loading motor " << motorName << " with type " << type);
                continue;
            }

            RCLCPP_DEBUG_STREAM(get_logger(), "Binding motor " << motorName);
            motors::MotorContainer motorContainer = {
                *motorBase,

                create_subscription<can_msgs::msg::MotorMsg>("motor/" + motorName + "/demand", rclcpp::SystemDefaultsQoS(),
                    std::bind(&motors::Motor::setValue, std::ref(motorContainer.motor), _1)),

                create_service<can_msgs::srv::SetPIDFGains>("motor/" + motorName + "/set_pidf",
                                                            std::bind(&motors::Motor::configMotorPIDF, std::ref(motorContainer.motor), _1, _2))};

            motorContainers.push_back(motorContainer);
        }

        RCLCPP_DEBUG(get_logger(), "Declaring motor parameters");
        for (auto motorContainer : motorContainers) {
            this->shared_from_this();
            motorContainer.motor.declareConfig(this->shared_from_this());
        }

        RCLCPP_DEBUG(get_logger(), "Configuring motors from parameters");
        for (auto motorContainer : motorContainers) {
            motorContainer.motor.executeConfig(this->shared_from_this());
        }

        // Allow timer to free run now
        highRate->reset();
    }

    void feedSafety(std::shared_ptr<std_msgs::msg::Bool> msg) {
        if (msg->data) ctre::phoenix::unmanaged::FeedEnable(SAFETY_TIMEOUT_MS);
    }

    void highRateCallback() {
        auto msg = sensor_msgs::msg::JointState();
        for (auto motor : motorContainers) {
            auto data = motor.motor.getJointState();
            msg.name.push_back(data.name);
            msg.position.push_back(data.position);
            msg.velocity.push_back(data.velocity);
            msg.effort.push_back(data.effort);
        }
        motorStatePub->publish(msg);
    }

    void can_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat) {
        // get bus statistics
        float busPct;
        uint32_t busOff, txFull, rec, tec;
        int32_t status;
        ctre::phoenix::platform::can::CANbus_GetStatus(&busPct, &busOff, &txFull, &rec, &tec, &status);

        auto rosStatus = diagnostic_msgs::msg::DiagnosticStatus::OK;
        if(busPct > 0.5)
            rosStatus = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        if(busPct > 0.9 || txFull > 25 || status != 0 || busOff > 25)
            rosStatus = diagnostic_msgs::msg::DiagnosticStatus::ERROR;

        stat.summary(
            rosStatus,
            "CAN Bus statistics");

        stat.add("Bus time percentage", busPct);
        stat.add("Bus off count", busOff);
        stat.add("Bus transmit buffer full error", txFull);
        stat.add("Bus rec", rec);
        stat.add("Bus tec", tec);
        stat.add("Bus Status", busPct);
    }

 private:
    // safety enable subscription that allows motors to be active
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr safetySubscrip;

    // publisher for the current joint states of each joint
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr motorStatePub;

    rclcpp::TimerBase::SharedPtr highRate;

    std::vector<motors::MotorContainer> motorContainers;

    std::shared_ptr<diagnostic_updater::Updater> updater;
};

int main(int argc, char **argv) {
    // init ros node
    rclcpp::init(argc, argv);

    // init canbus
    std::string interface = "can0";
    ctre::phoenix::platform::can::SetCANInterface(interface.c_str());

    auto node = std::make_shared<HardwareNode>();
    node->createMotors();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
