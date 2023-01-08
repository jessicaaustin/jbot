#include <utility>

#include "jbot_autonomy/jbot_autonomy.hpp"

//
// TREE
//

SetBlackboardInputs::SetBlackboardInputs(const std::string &name,
                                         const BT::NodeConfiguration &config,
                                         const std::shared_ptr<JBotAutonomyNode> &nh)
        : SyncActionNode(name, config), nh_(nh) {}

BT::PortsList SetBlackboardInputs::providedPorts() {
    return {
            BT::OutputPort<std::string>("bot_mode"),
            BT::OutputPort<int>("forward_range_cm"),
            // TODO specify type (https://www.behaviortree.dev/docs/3.8/tutorial-basics/tutorial_03_generic_ports)
            BT::OutputPort<std::string>("goal_waypoint")};
}

BT::NodeStatus SetBlackboardInputs::tick() {
    setOutput("bot_mode", nh_->get_bot_mode());
    // TODO set actual value
    setOutput("forward_range_cm", 99);
    // TODO set actual value
    setOutput("goal_waypoint", "1 0");
    return BT::NodeStatus::SUCCESS;
}

BotModeConditionNode::BotModeConditionNode(const std::string &name,
                                           const BT::NodeConfiguration &config,
                                           std::string target_bot_mode)
        : SyncActionNode(name, config), target_bot_mode_(std::move(target_bot_mode)) {}

BT::PortsList BotModeConditionNode::providedPorts() {
    return {BT::InputPort<std::string>("bot_mode")};
}

BT::NodeStatus BotModeConditionNode::tick() {
    BT::Optional<std::string> bot_mode = getInput<std::string>("bot_mode");
    if (!bot_mode) {
        throw BT::RuntimeError("missing required input [bot_mode]: ",
                               bot_mode.error());
    }
    return bot_mode == target_bot_mode_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

StopMotion::StopMotion(const std::string &name,
                       const std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::TwistStamped>> &cmd_vel_pub)
        : SyncActionNode(name, {}), cmd_vel_pub_(cmd_vel_pub) {}

BT::NodeStatus StopMotion::tick() {
    // TODO use node logger
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Publishing 'zero' Twist on /cmd_vel");
    auto message = geometry_msgs::msg::TwistStamped();
    // TODO set header stamp on message
    this->cmd_vel_pub_->publish(message);
    return BT::NodeStatus::SUCCESS;
}

OperatorMove::OperatorMove(const std::string &name) : BT::SyncActionNode(name, {}) {}

BT::NodeStatus OperatorMove::tick() {
    // TODO use node logger
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Publishing /operator_command.teleop_cmd Twist on /cmd_vel");
    // TODO implement
    return BT::NodeStatus::SUCCESS;
}

GoalMove::GoalMove(const std::string &name) : BT::SyncActionNode(name, {}) {}

BT::NodeStatus GoalMove::tick() {
    // TODO use node logger
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Publishing Twist to reach /operator_command.goal, "
                "given current /robot_state.pose");
    // TODO implement
    return BT::NodeStatus::SUCCESS;
}

//
// ROS NODE
//

void registerBotModeConditionNode(BT::BehaviorTreeFactory &factory,
                                  const char *node_name, const std::string &bot_mode) {
    factory.registerBuilder<BotModeConditionNode>(node_name,
                                                  [&](const std::string &name, const BT::NodeConfiguration &config) {
                                                      return std::make_unique<BotModeConditionNode>(name, config,
                                                                                                    bot_mode);
                                                  });
}

JBotAutonomyNode::JBotAutonomyNode() : Node("jbot_autonomy") {
    op_cmd_ = std::make_unique<jbot_interfaces::msg::OperatorCommand>();
    op_cmd_->bot_mode = BOT_MODE_IDLE;

    bot_mode_pub_ = this->create_publisher<std_msgs::msg::String>("bot_mode", 10);
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 10);
    op_cmd_sub_ = this->create_subscription<jbot_interfaces::msg::OperatorCommand>(
            "operator_command", 10,
            [&](jbot_interfaces::msg::OperatorCommand::UniquePtr msg) {
                // TODO why isn't this callback getting called?
                // TODO is this best practice? and use sharedptr or uniqueptr?
                op_cmd_ = std::move(msg);
            }
    );
}

void JBotAutonomyNode::create_tree(const std::shared_ptr<JBotAutonomyNode> nh) {

    BT::BehaviorTreeFactory factory;
    factory.registerBuilder<SetBlackboardInputs>("SetBlackboardInputs",
                                                 [nh](const std::string &name,
                                                        const BT::NodeConfiguration &config) {
                                                     return std::make_unique<SetBlackboardInputs>(name, config, nh);
                                                 });
    factory.registerNodeType<OperatorMove>("OperatorMove");
    factory.registerNodeType<GoalMove>("GoalMove");
    factory.registerBuilder<StopMotion>("StopMotion",
                                        [&](const std::string &name,
                                            __attribute__((unused)) const BT::NodeConfiguration &config) {
                                            return std::make_unique<StopMotion>(name, cmd_vel_pub_);
                                        });
    registerBotModeConditionNode(factory, "IsBotModeIdle", BOT_MODE_IDLE);
    registerBotModeConditionNode(factory, "IsBotModeTeleop", BOT_MODE_TELEOP);
    registerBotModeConditionNode(factory, "IsBotModeAutonomous", BOT_MODE_AUTONOMOUS);
    tree_ = factory.createTreeFromFile(BT_XML_PATH);
}

std::string JBotAutonomyNode::get_bot_mode() {
    return op_cmd_->bot_mode;
}

void JBotAutonomyNode::run() {

    RCLCPP_INFO(this->get_logger(), "Starting execution");

    // Run through tree over and over until user stops the process or the tree fails.
    rclcpp::Rate loop_rate(10);
    while (rclcpp::ok()) {
        // Publish current state
        auto bot_mode_msg = std_msgs::msg::String();
        bot_mode_msg.data = this->op_cmd_->bot_mode;
        this->bot_mode_pub_->publish(bot_mode_msg);

        // Run tree once.
        // If the tree fails, then break.
        try {
            BT::NodeStatus status = tree_.tickRoot();
            if (status != BT::NodeStatus::SUCCESS) {
                RCLCPP_ERROR(this->get_logger(),
                             "Tree execution failed! Status = %d", (int) status);
                break;
            }
        } catch (BT::RuntimeError &e) {
            RCLCPP_ERROR(this->get_logger(),
                         "Tree execution failed! Error = %s", e.what());
            break;
        }
        loop_rate.sleep();
    }

    rclcpp::shutdown();
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JBotAutonomyNode>();
    node->create_tree(node);
    node->run();
    rclcpp::shutdown();
    return 0;
}