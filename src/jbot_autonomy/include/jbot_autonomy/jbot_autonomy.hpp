#ifndef SRC_JBOT_AUTONOMY_H
#define SRC_JBOT_AUTONOMY_H

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "jbot_interfaces/msg/operator_command.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"

// TODO pass in via launch file or ros param or something
#define BT_XML_PATH "/home/jessica/dev/personal/jbot_ws/src/jbot_autonomy/config/jbot_autonomy_tree.xml"

const std::string BOT_MODE_IDLE = "idle";
const std::string BOT_MODE_TELEOP = "teleop";
const std::string BOT_MODE_AUTONOMOUS = "autonomous";

//
// TREE
//

/**
 * @brief Set the current values for input ports to the blackboard (bot_mode, etc).
 */
class SetBlackboardInputs : public BT::SyncActionNode {
public:
    SetBlackboardInputs(const std::string &name, const BT::NodeConfiguration &config);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;
};

/**
 * @brief Publish "zero" Twist on /cmd_vel.
 */
class StopMotion : public BT::SyncActionNode {
public:
    explicit StopMotion(const std::string &name,
                        const std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::TwistStamped>> &cmd_vel_pub);

    BT::NodeStatus tick() override;

private:
    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::TwistStamped>> cmd_vel_pub_;
};

/**
 * @brief Publish operator_command Twist on /cmd_vel.
 */
class OperatorMove : public BT::SyncActionNode {
public:
    explicit OperatorMove(const std::string &name);

    BT::NodeStatus tick() override;
};

/**
 * @brief Publish Twist on /cmd_vel to achieve operator_command goal, given current bot pose.
 */
class GoalMove : public BT::SyncActionNode {
public:
    explicit GoalMove(const std::string &name);

    BT::NodeStatus tick() override;
};


/**
 * @brief ConditionNode based on current bot_mode value.
 *        Returns SUCCESS if bot_mode equals target_bot_mode, FAILURE otherwise.
 */
class BotModeConditionNode : public BT::SyncActionNode {
public:
    BotModeConditionNode(const std::string &name, const BT::NodeConfiguration &config,
                         std::string target_bot_mode);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

private:
    std::string target_bot_mode_;
};


//
// ROS NODE
//

class JBotAutonomyNode : public rclcpp::Node {
public:
    /**
     * Initialize pub, sub, and behavior tree.
     */
    JBotAutonomyNode();

    /**
     * Evaluate the tree in a loop.
     */
    void run();

private:
    jbot_interfaces::msg::OperatorCommand::UniquePtr op_cmd_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr bot_mode_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<jbot_interfaces::msg::OperatorCommand>::SharedPtr op_cmd_sub_;
    BT::Tree tree_;

};

#endif //SRC_JBOT_AUTONOMY_H
