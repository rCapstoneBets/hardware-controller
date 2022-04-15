#include "motor.hpp"

#define Phoenix_No_WPI  // remove WPI dependencies
#include "ctre/Phoenix.h"

namespace motors {
    class TalonBrushed : public Motor {
        public:
        TalonBrushed(int id, std::string motorName) {
            motor = std::make_shared<TalonSRX>(id);
            name = motorName;
        }

        void configMotorPIDF(const std::shared_ptr<can_msgs::srv::SetPIDFGains::Request> req,
                             std::shared_ptr<can_msgs::srv::SetPIDFGains::Response> resp) override {
            std::cout << "config PIDF for " << name << " motor" << std::endl;
            motor->SelectProfileSlot(req->pid_slot, 0);
            motor->Config_kP(req->pid_slot, req->k_p, 0);
            motor->Config_kI(req->pid_slot, req->k_i, 0);
            motor->Config_IntegralZone(req->pid_slot, req->i_max, 0);
            motor->Config_kD(req->pid_slot, req->k_d, 0);
            motor->Config_kF(req->pid_slot, req->k_f, 0);
            //resp->success = motor->GetLastError() == OK;
            resp->success = true;
        }

        void declareConfig(std::shared_ptr<rclcpp::Node> node) override {
            RCLCPP_DEBUG_STREAM(node->get_logger(), "Declaring motor params for " << name);

            // Basic params
            node->declare_parameter<bool>(name + ".inverted", false);
            node->declare_parameter<bool>(name + ".brake_mode", false);
            node->declare_parameter<int>(name + ".follower", -1);
            node->declare_parameter<double>(name + ".vcomp", -1);

            // PID params
            node->declare_parameter<double>(name + ".pid.kp", 0.0);
            node->declare_parameter<double>(name + ".pid.ki", 0.0);
            node->declare_parameter<double>(name + ".pid.kd", 0.0);
            node->declare_parameter<double>(name + ".pid.kf", 0.0);
            node->declare_parameter<double>(name + ".pid.izone", 0.0);

            // motion magic params
            node->declare_parameter<double>(name + ".motion_magic.cruise_vel", 0.0);
            node->declare_parameter<double>(name + ".motion_magic.accel", 0.0);
            node->declare_parameter<int>(name + ".motion_magic.smoothing", -1);

            // Current limit params
            node->declare_parameter<bool>(name + ".current_lim.enable", -1);
            node->declare_parameter<double>(name + ".current_lim.abs_lim", -1);
            node->declare_parameter<double>(name + ".current_lim.lim_trigger", -1);
            node->declare_parameter<double>(name + ".current_lim.time_window", -1);
        }

        void executeConfig(std::shared_ptr<rclcpp::Node> node) override {
            RCLCPP_DEBUG_STREAM(node->get_logger(), "Configuring motor params for " << name);

            // Set basic settings
            motor->ConfigFactoryDefault();
            motor->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0);
            motor->SelectProfileSlot(0, 0);
            motor->SetSelectedSensorPosition(0.0);
            motor->SetInverted(node->get_parameter(name + ".inverted").as_bool());

            // Configure follower mode (assumes other device is another falcon)
            auto follower = node->get_parameter(name + ".follower").as_int();
            if (follower >= 0)
                motor->Set(ctre::phoenix::motorcontrol::ControlMode::Follower, follower);

            // Configure the neutral mode
            motor->SetNeutralMode(node->get_parameter(name + ".brake_mode").as_bool()
                                      ? NeutralMode::Brake
                                      : NeutralMode::Coast);

            // Configure voltage compensation
            auto vcomp = node->get_parameter(name + ".vcomp").as_double();
            if (vcomp > 0.0) {
                motor->ConfigVoltageCompSaturation(vcomp);
                motor->EnableVoltageCompensation(true);
            }

            auto motionVel = node->get_parameter(name + ".motion_magic.cruise_vel").as_double() / 10.0 * 1024.0 / M_PI;
            auto motionAccel = node->get_parameter(name + ".motion_magic.accel").as_double() / 10.0 * 1024.0 / M_PI;
            auto motionSmooth = node->get_parameter(name + ".motion_magic.smoothing").as_int();

            if(motionVel > 0 && motionAccel > 0 && motionSmooth > -1){
                motor->ConfigMotionCruiseVelocity(motionVel);
                motor->ConfigMotionAcceleration(motionAccel);
                motor->ConfigMotionSCurveStrength(motionSmooth);
            } else if (motionVel > 0 || motionAccel > 0 || motionSmooth > -1){
                RCLCPP_WARN_STREAM(node->get_logger(), name + " is missing one or more of the values to turn motion magic on! cruise: " 
                << motionVel << " accel: " << motionAccel << " smoothing: " << motionSmooth);
            }
            
            // Configure PID settings
            motor->Config_kP(0, node->get_parameter(name + ".pid.kp").as_double(), 0);
            motor->Config_kI(0, node->get_parameter(name + ".pid.ki").as_double(), 0);
            motor->Config_kD(0, node->get_parameter(name + ".pid.kd").as_double(), 0);
            motor->Config_kF(0, node->get_parameter(name + ".pid.kf").as_double(), 0);
            motor->Config_IntegralZone(0, node->get_parameter(name + ".pid.izone").as_double(), 0);

            // Configure Current Limiting
            auto currentLimEnable = node->get_parameter(name + ".current_lim.enable").as_bool();
            auto currentLimitVal = node->get_parameter(name + ".current_lim.abs_lim").as_double();
            auto currentLimitTrigger = node->get_parameter(name + ".current_lim.lim_trigger").as_double();
            auto currentLimitTime = node->get_parameter(name + ".current_lim.time_window").as_double();

            motor->EnableCurrentLimit(currentLimEnable);
            motor->ConfigPeakCurrentDuration(currentLimitTime * 1000);
            motor->ConfigPeakCurrentLimit(currentLimitTrigger);
            motor->ConfigContinuousCurrentLimit(currentLimitVal);
        }

        void setValue(const can_msgs::msg::MotorMsg::SharedPtr msg) override {
            //std::cout << "recieved data for " << name << " motor" << std::endl;
            auto controlMode = static_cast<ctre::phoenix::motorcontrol::ControlMode>(msg->control_mode);
            switch(controlMode){
                case ControlMode::PercentOutput :
                    //std::cout << "running percent ouptut, demand: "  << msg->demand << std::endl;
                    motor->Set(controlMode, msg->demand);
                    break;

                case ControlMode::Velocity :
                    motor->Set(controlMode, msg->demand / 10.0 * 1024.0 / M_PI, DemandType::DemandType_ArbitraryFeedForward, msg->arb_feedforward);
                    break;

                case ControlMode::Position :
                    motor->Set(controlMode, msg->demand * 1024.0 / M_PI, DemandType::DemandType_ArbitraryFeedForward, msg->arb_feedforward);
                    break;

                case ControlMode::MotionMagic : 
                    motor->Set(controlMode, msg->demand * 1024.0 / M_PI, DemandType::DemandType_ArbitraryFeedForward, msg->arb_feedforward);
                    break;
                
                default:
                    std::cout << "Unknown / unused control mode!" << std::endl;

            }
            
        }

        void setSensorPos(const std_msgs::msg::Float32::SharedPtr msg) override {
            motor->SetSelectedSensorPosition(msg->data);
        }

        JointState getJointState(){
            return {
                name,
                motor->GetSelectedSensorPosition() / 1024.0 * M_PI,
                motor->GetSelectedSensorVelocity() * 10.0 / 1024.0 * M_PI,
                motor->GetStatorCurrent(),
            };
        }

     private:
        std::shared_ptr<TalonSRX> motor;
        std::string name;
    };
}