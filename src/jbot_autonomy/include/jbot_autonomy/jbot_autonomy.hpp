#ifndef SRC_JBOT_AUTONOMY_H
#define SRC_JBOT_AUTONOMY_H

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"

// TODO pass in via launch file or ros param or something
#define BT_XML_PATH "/home/jessica/dev/personal/jbot_ws/src/jbot_autonomy/config/jbot_autonomy_tree.xml"


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
                        const std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist>> &cmd_vel_pub);

    BT::NodeStatus tick() override;

private:
    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist>> cmd_vel_pub_;
};

//
///**
// * @brief ConditionNode based on current bot_mode value.
// */
//class BotModeConditionBase : public BT::SyncActionNode {
//public:
//    BotModeConditionBase(const std::string &name, const BT::NodeConfiguration &config);
//
//    static BT::PortsList providedPorts();
//
//    BT::NodeStatus tick() override;
//};


class IsBotModeIdle : public BT::SyncActionNode {
public:
    IsBotModeIdle(const std::string &name, const BT::NodeConfiguration &config);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;
};


#endif //SRC_JBOT_AUTONOMY_H
