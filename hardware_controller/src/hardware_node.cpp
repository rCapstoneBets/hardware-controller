#include <can_msgs/msg/motor_msg.hpp>
#include <cstdio>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>

#include "rclcpp/rclcpp.hpp"

#define Phoenix_No_WPI  // remove WPI dependencies
#include "ctre/Phoenix.h"
#include "ctre/phoenix/cci/Unmanaged_CCI.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"

#define SAFETY_TIMEOUT_MS 100
using std::placeholders::_1;
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

        RCLCPP_DEBUG(get_logger(), "Initializing subscribers");

        safetySubscrip = create_subscription<std_msgs::msg::Bool>(
            "safety_enable", rclcpp::SystemDefaultsQoS(),
            std::bind(&HardwareNode::feedSafety, this, _1), subsOpt);

        panMotorSub = create_subscription<can_msgs::msg::MotorMsg>(
            "motor/pan", rclcpp::SystemDefaultsQoS(),
            std::bind(&HardwareNode::updatePanMotor, this, _1), subsOpt);

        tiltMotorSub = create_subscription<can_msgs::msg::MotorMsg>(
            "motor/tilt", rclcpp::SystemDefaultsQoS(),
            std::bind(&HardwareNode::updateTiltMotor, this, _1), subsOpt);

        wheelMotorSub = create_subscription<can_msgs::msg::MotorMsg>(
            "motor/wheel", rclcpp::SystemDefaultsQoS(),
            std::bind(&HardwareNode::updateWheelMotor, this, _1), subsOpt);

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
        panMotor = std::make_shared<TalonFX>(panMotorId);
        auto tiltMotorId = this->get_parameter("tilt_motor.id").as_int();
        tiltMotor = std::make_shared<TalonFX>(tiltMotorId);
        auto leftWheelMotorId =
            this->get_parameter("left_wheel_motor.id").as_int();
        leftWheelMotor = std::make_shared<TalonFX>(leftWheelMotorId);
        auto rightWheelMotorId =
            this->get_parameter("right_wheel_motor.id").as_int();
        rightWheelMotor = std::make_shared<TalonFX>(rightWheelMotorId);

        RCLCPP_DEBUG(get_logger(), "Configuring motors");

        configMotor(panMotor, "pan_motor");
        configMotor(tiltMotor, "tilt_motor");
        configMotor(leftWheelMotor, "left_motor");
        configMotor(rightWheelMotor, "right_motor");

        RCLCPP_DEBUG(get_logger(), "Initializing timers");

        highRate = create_wall_timer(
            10ms, std::bind(&HardwareNode::highRateCallback, this), pubs);

        RCLCPP_INFO(get_logger(), "Hardware controller initalized");
    }

    void configMotor(std::shared_ptr<TalonFX> motor, std::string motorName) {
        RCLCPP_DEBUG_STREAM(get_logger(), "Configuring motor " << motorName);

        // Basic params
        this->declare_parameter<bool>(motorName + ".inverted", false);
		this->declare_parameter<bool>(motorName + ".brake_mode", false);
        this->declare_parameter<int>(motorName + ".follower", -1);
        this->declare_parameter<double>(motorName + ".vcomp", -1);

		// PID params
        this->declare_parameter<double>(motorName + ".pid.kp", 0.0);
		this->declare_parameter<double>(motorName + ".pid.ki", 0.0);
		this->declare_parameter<double>(motorName + ".pid.kd", 0.0);
		this->declare_parameter<double>(motorName + ".pid.kf", 0.0);
		this->declare_parameter<double>(motorName + ".pid.izone", 0.0);

		// Current limit params
		this->declare_parameter<bool>(motorName + ".current_lim.enable", -1);
		this->declare_parameter<double>(motorName + ".current_lim.abs_lim", -1);
		this->declare_parameter<double>(motorName + ".current_lim.lim_trigger", -1);
        this->declare_parameter<double>(motorName + ".current_lim.time_window", -1);

		// Set basic settings
        motor->ConfigFactoryDefault();
        motor->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor);
        motor->SelectProfileSlot(0, 0);
        motor->SetInverted(this->get_parameter(motorName + ".inverted").as_bool());

		// Configure follower mode (assumes other device is another falcon)
        auto follower = get_parameter(motorName + ".follower").as_int();
        if (follower >= 0)
            motor->Set(ctre::phoenix::motorcontrol::ControlMode::Follower, follower);

        // Configure the neutral mode
        motor->SetNeutralMode(get_parameter(motorName + ".brake_mode").as_bool()
                                  ? NeutralMode::Brake : NeutralMode::Coast);

		// Configure voltage compensation
        auto vcomp = get_parameter(motorName + ".vcomp").as_double();
        if (vcomp > 0.0) {
            motor->ConfigVoltageCompSaturation(vcomp);
            motor->EnableVoltageCompensation(true);
        }

		// Configure PID settings
        motor->Config_kP(0, get_parameter(motorName + ".pid.kp").as_double(), 0);
        motor->Config_kI(0, get_parameter(motorName + ".pid.ki").as_double(), 0);
        motor->Config_kD(0, get_parameter(motorName + ".pid.kd").as_double(), 0);
        motor->Config_kF(0, get_parameter(motorName + ".pid.kf").as_double(), 0);
        motor->Config_IntegralZone(0, get_parameter(motorName + ".pid.izone").as_double(), 0);

		// Configure Current Limiting
        auto currentLimEnable = get_parameter(motorName + ".current_lim.enable").as_bool(); 
        auto currentLimitVal = get_parameter(motorName + ".current_lim.abs_lim").as_double();
        auto currentLimitTrigger = get_parameter(motorName + ".current_lim.lim_trigger").as_double();	
        auto currentLimitTime = get_parameter(motorName + ".current_lim.time_window").as_double();
        motor->ConfigStatorCurrentLimit({currentLimEnable, currentLimitVal,
                                         currentLimitTrigger,
                                         currentLimitTime});
    }

    void feedSafety(std::shared_ptr<std_msgs::msg::Bool> msg) {
        if (msg->data) ctre::phoenix::unmanaged::FeedEnable(SAFETY_TIMEOUT_MS);
    }

    void highRateCallback() {}

    void updatePanMotor(std::shared_ptr<can_msgs::msg::MotorMsg> msg) {}

    void updateTiltMotor(std::shared_ptr<can_msgs::msg::MotorMsg> msg) {}

    void updateWheelMotor(std::shared_ptr<can_msgs::msg::MotorMsg> msg) {}

 private:
    // safety enable subscription that allows motors to be active
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr safetySubscrip;

    // subscribers to take desired state for each motor
    rclcpp::Subscription<can_msgs::msg::MotorMsg>::SharedPtr panMotorSub,
        tiltMotorSub, wheelMotorSub;

    // publisher for the current joint states of each joint
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr motorStatePub;

    rclcpp::CallbackGroup::SharedPtr subs, pubs;
    rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>> subsOpt;

    rclcpp::TimerBase::SharedPtr highRate;

    std::shared_ptr<TalonFX> panMotor, tiltMotor, leftWheelMotor,
        rightWheelMotor;
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
