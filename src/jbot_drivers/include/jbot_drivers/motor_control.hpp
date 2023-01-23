#ifndef BUILD_MOTOR_CONTROL_HPP
#define BUILD_MOTOR_CONTROL_HPP

#include <cppgpio.hpp>
#include "rclcpp/rclcpp.hpp"
#include "jbot_interfaces/msg/motor_state.hpp"

// https://pinout.xyz/pinout/wiringpi
#define MOTOR_1A 1 // GPIO18
#define MOTOR_1B 26 // GPIO12
#define MOTOR_2A 23 // GPIO13
#define MOTOR_2B 24 // GPIO19
#define MAX_RPM 10  // TODO determine this
#define PWM_RANGE 100

/**
 * Handles executing motor commands, and publishing motor state.
 *
 * TODO: break out code that talks to motors in a separate class, to make unit testing easier.
 */
class MotorControlNode : public rclcpp::Node {
public:
    MotorControlNode();

private:
    /**
     * Latest motor RPM values
     */
    int left_rpm_;
    int right_rpm_;
    /**
     * Most recent incoming command
     */
    jbot_interfaces::msg::MotorState::SharedPtr motor_cmd_;
    std::chrono::milliseconds watchdog_timeout_;

    rclcpp::Subscription<jbot_interfaces::msg::MotorState>::SharedPtr motor_cmd_sub_;
    rclcpp::Publisher<jbot_interfaces::msg::MotorState>::SharedPtr motor_state_pub_;

    /**
     * Whether or not the motors have been initialized.
     */
    bool initialized_;

    void initialize_motors();

    void update_motor_pwm(int left_rpm, int right_rpm);

    rclcpp::TimerBase::SharedPtr run_timer_;

    /**
     * One execution loop, to be executed periodically by the run_timer.
     * Updates the motor PWM value, then publishes diagnostics and motor_state.
     */
    void run_once();
};


#endif //BUILD_MOTOR_CONTROL_HPP
