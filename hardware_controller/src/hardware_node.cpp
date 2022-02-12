#include <can_msgs/msg/motor_msg.hpp>
#include <can_msgs/srv/set_pidf_gains.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>

#define Phoenix_No_WPI  // remove WPI dependencies
#include "ctre/Phoenix.h"
#include "ctre/phoenix/cci/Unmanaged_CCI.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"

#include "motor.hpp"
#include "talon_brushless.hpp"

#include <algorithm>
#include <cctype>
#include <string>

#define SAFETY_TIMEOUT_MS 100
using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;


class HardwareNode : public rclcpp::Node {
 public:
    HardwareNode() : Node("hardware_node") {
        // Register publisher ans subscriber callback groups
        subs = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        pubs = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        subsOpt = rclcpp::SubscriptionOptions();
        subsOpt.callback_group = subs;

		RCLCPP_DEBUG(get_logger(), "Loading motors from config");

		this->declare_parameter<std::vector<std::string>>("motor_names", {});
		auto motorNames = this->get_parameter("pan_motor.id").as_string_array();

        for(auto motorName : motorNames){
			RCLCPP_DEBUG_STREAM(get_logger(), "Building motor " << motorName);
			this->declare_parameter<int>(motorName + ".id", 0);
			this->declare_parameter<std::string>(motorName + ".type", "");

			auto type = this->get_parameter(motorName + ".type").as_string();
			std::transform(type.begin(), type.end(), type.begin(), 
				[](unsigned char c){return std::tolower(c);});

			motors::MotorContainer motorContainer;
			if(type.find("talonfx") != std::string::npos){
				motorContainer.motor = motors::TalonBrushless(
					this->get_parameter(motorName +".id").as_int(),
					motorName
				);
			} else {
				RCLCPP_ERROR_STREAM(get_logger(), "Error loading motor " << motorName << " with type " << type);
			}

			RCLCPP_DEBUG(get_logger(), "Initializing subscription");

			motorContainer.sub = create_subscription<can_msgs::msg::MotorMsg>(
            "motor/" + motorName, rclcpp::SystemDefaultsQoS(),
            std::bind(&motors::Motor::setValue, motorContainer.motor, _1), subsOpt);

			RCLCPP_DEBUG(get_logger(), "Initializing PIDF service");

			motorContainer.pidfSrv = create_service<can_msgs::srv::SetPIDFGains>("motor/" + motorName + "/set_pidf", 
			std::bind(&motors::Motor::setValue, motorContainer.motor, _1, _2), subs);

		}

        safetySubscrip = create_subscription<std_msgs::msg::Bool>(
            "safety_enable", rclcpp::SystemDefaultsQoS(),
            std::bind(&HardwareNode::feedSafety, this, _1), subsOpt);

    

        

        RCLCPP_DEBUG(get_logger(), "Initializing publishers");

        motorStatePub = create_publisher<sensor_msgs::msg::JointState>(
            "motor/joint_state", rclcpp::SensorDataQoS());

        RCLCPP_DEBUG(get_logger(), "Getting parameters");

        this->declare_parameter<int>("pan_motor.id", 1);
        this->declare_parameter<int>("tilt_motor.id", 2);
        this->declare_parameter<int>("left_wheel_motor.id", 3);
        this->declare_parameter<int>("right_wheel_motor.id", 4);

        RCLCPP_DEBUG(get_logger(), "Constructing motors");

        auto panMotorId = this->get_parameter("pan_motor.id").as_int();
        auto tiltMotorId = this->get_parameter("tilt_motor.id").as_int();
        auto leftWheelMotorId = this->get_parameter("left_wheel_motor.id").as_int();
        auto rightWheelMotorId = this->get_parameter("right_wheel_motor.id").as_int();

        panMotor = std::make_shared<TalonFX>(panMotorId);
        tiltMotor = std::make_shared<TalonFX>(tiltMotorId);
        leftWheelMotor = std::make_shared<TalonFX>(leftWheelMotorId);
        rightWheelMotor = std::make_shared<TalonFX>(rightWheelMotorId);

        RCLCPP_DEBUG(get_logger(), "Configuring motors");

        configMotor(panMotor, "pan_motor");
        configMotor(tiltMotor, "tilt_motor");
        configMotor(leftWheelMotor, "left_wheel_motor");
        configMotor(rightWheelMotor, "right_wheel_motor");

        RCLCPP_DEBUG(get_logger(), "Initializing timers");

        highRate = create_wall_timer(
            10ms, std::bind(&HardwareNode::highRateCallback, this), pubs);

        RCLCPP_INFO(get_logger(), "Hardware controller initalized");
    }

    void feedSafety(std::shared_ptr<std_msgs::msg::Bool> msg) {
        if (msg->data) ctre::phoenix::unmanaged::FeedEnable(SAFETY_TIMEOUT_MS);
    }

    void highRateCallback() {
        auto msg = sensor_msgs::msg::JointState();
        
    }

    void updatePanMotor(std::shared_ptr<can_msgs::msg::MotorMsg> msg) {}

    void updateTiltMotor(std::shared_ptr<can_msgs::msg::MotorMsg> msg) {}

    void updateWheelMotor(std::shared_ptr<can_msgs::msg::MotorMsg> msg) {}

    

 private:
    // safety enable subscription that allows motors to be active
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr safetySubscrip;

    // publisher for the current joint states of each joint
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr motorStatePub;

    rclcpp::CallbackGroup::SharedPtr subs, pubs;
    rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>> subsOpt;

    rclcpp::TimerBase::SharedPtr highRate;

    std::vector<motors::MotorContainer> motorContainers;

};

int main(int argc, char **argv) {
    // init ros node
    rclcpp::init(argc, argv);

    // init canbus
    std::string interface = "can0";
    ctre::phoenix::platform::can::SetCANInterface(interface.c_str());

    rclcpp::executors::MultiThreadedExecutor executor;

    auto node = std::make_shared<HardwareNode>();

    executor.add_node(node);

    // serve the callbacks
    executor.spin();

    rclcpp::shutdown();

    return 0;
}
