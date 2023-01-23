#include "jbot_drivers/motor_control.hpp"
#include <wiringPi.h>
#include <softPwm.h>

MotorControlNode::MotorControlNode() : Node("motor_control"),
                                       left_rpm_(0), right_rpm_(0), initialized_(false) {
    this->declare_parameter("main_loop_period_ms");
    this->declare_parameter("watchdog_timeout_ms");

    // TODO: shared constants for topic names
    motor_cmd_ = std::make_unique<jbot_interfaces::msg::MotorState>();
    motor_cmd_sub_ = this->create_subscription<jbot_interfaces::msg::MotorState>(
            "motor_cmd", 10,
            [&](jbot_interfaces::msg::MotorState::SharedPtr msg) {
                motor_cmd_ = msg;
            });
    motor_state_pub_ = this->create_publisher<jbot_interfaces::msg::MotorState>("motor_state", 10);

    std::chrono::milliseconds timer_period(this->get_parameter("main_loop_period_ms").get_parameter_value().get<int>());
    run_timer_ = this->create_wall_timer(timer_period, std::bind(&MotorControlNode::run_once, this));

    watchdog_timeout_ = std::chrono::milliseconds{
            this->get_parameter("watchdog_timeout_ms").get_parameter_value().get<int>()};

}

void MotorControlNode::initialize_motors() {
    if (wiringPiSetup() != 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed wiringPiSetup");
        return;
    }

    int initial_value = 0;
    for (const auto &motor_pin: {MOTOR_1A, MOTOR_1B, MOTOR_2A, MOTOR_2B}) {
        pinMode(motor_pin, OUTPUT);
        if (softPwmCreate(motor_pin, initial_value, PWM_RANGE) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed PWM setup for motor pin %d", motor_pin);
            return;
        }
    }

    RCLCPP_INFO(this->get_logger(), "Motors initialized");
    initialized_ = true;
}

void MotorControlNode::update_motor_pwm(int left_rpm, int right_rpm) {
    //  State  A  B
    //  FWD    0  1
    //  REV    1  0
    //  STOP   0  0
    //  short  1  1

    int left_pwm = (int) ((left_rpm / MAX_RPM) * PWM_RANGE);
    int right_pwm = (int) ((right_rpm / MAX_RPM) * PWM_RANGE);

    if (left_pwm > 0) {
        softPwmWrite(MOTOR_1A, LOW);
        softPwmWrite(MOTOR_1B, left_pwm);
    } else {
        softPwmWrite(MOTOR_1A, left_pwm);
        softPwmWrite(MOTOR_1B, LOW);
    }
    if (right_pwm > 0) {
        softPwmWrite(MOTOR_2A, LOW);
        softPwmWrite(MOTOR_2B, right_pwm);
    } else {
        softPwmWrite(MOTOR_2A, right_pwm);
        softPwmWrite(MOTOR_2B, LOW);
    }
}

void MotorControlNode::run_once() {
    if (!initialized_) {
        this->initialize_motors();
    }

    if (initialized_) {
        int motor_cmd_left = 0;
        int motor_cmd_right = 0;

        std::chrono::nanoseconds time_since_command(
                (this->get_clock()->now() - motor_cmd_->header.stamp).nanoseconds());
        if (time_since_command < watchdog_timeout_) {
            motor_cmd_left = motor_cmd_->left_rpm;
            motor_cmd_right = motor_cmd_->right_rpm;
        }

        this->update_motor_pwm(motor_cmd_left, motor_cmd_right);

        // Publish motor state
        // NOTE: since there are no wheel encoders, this simply publishes the executed command
        auto motor_state_msg = jbot_interfaces::msg::MotorState();
        motor_state_msg.header.stamp = this->get_clock()->now();
        motor_state_msg.left_rpm = motor_cmd_left;
        motor_state_msg.right_rpm = motor_cmd_right;
        this->motor_state_pub_->publish(motor_state_msg);

    }

    // TODO publish diagnostic for initialized_

}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorControlNode>());
    rclcpp::shutdown();
    return 0;
}