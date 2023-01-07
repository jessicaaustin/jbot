#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "jbot_autonomy/jbot_autonomy.hpp"

SetBlackboardInputs::SetBlackboardInputs(const std::string &name, const BT::NodeConfiguration &config)
        : SyncActionNode(name, config) {}

BT::PortsList SetBlackboardInputs::providedPorts() {
    return {
            // TODO bot_mode should be an enum? at least internally in this class
            BT::OutputPort<std::string>("bot_mode"),
            BT::OutputPort<int>("forward_range_cm"),
            // TODO specify type (https://www.behaviortree.dev/docs/3.8/tutorial-basics/tutorial_03_generic_ports)
            BT::OutputPort<std::string>("goal_waypoint")};
}

BT::NodeStatus SetBlackboardInputs::tick() {
    // TODO set based on class internal state
    setOutput("bot_mode", "idle");
    setOutput("forward_range_cm", 99);
    setOutput("goal_waypoint", "1 0");
    return BT::NodeStatus::SUCCESS;
}

IsBotModeIdle::IsBotModeIdle(const std::string &name, const BT::NodeConfiguration &config)
        : SyncActionNode(name, config) {}

BT::PortsList IsBotModeIdle::providedPorts() {
    return {BT::InputPort<std::string>("bot_mode")};
}

BT::NodeStatus IsBotModeIdle::tick() {
    BT::Optional<std::string> bot_mode = getInput<std::string>("bot_mode");
    if (!bot_mode) {
        throw BT::RuntimeError("missing required input [bot_mode]: ",
                               bot_mode.error());
    }
    return bot_mode == "idle" ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

StopMotion::StopMotion(const std::string &name,
                       const std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist>> &cmd_vel_pub)
        : SyncActionNode(name, {}), cmd_vel_pub_(cmd_vel_pub) {}

BT::NodeStatus StopMotion::tick() {
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Publishing 'zero' Twist on /cmd_vel");
    auto message = geometry_msgs::msg::Twist();
    this->cmd_vel_pub_->publish(message);
    return BT::NodeStatus::SUCCESS;
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("jbot_autonomy");
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub
            = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<SetBlackboardInputs>("SetBlackboardInputs");
    factory.registerNodeType<IsBotModeIdle>("IsBotModeIdle");
    // https://www.behaviortree.dev/docs/3.8/tutorial-basics/tutorial_08_additional_args
    factory.registerBuilder<StopMotion>("StopMotion",
                                        [cmd_vel_pub](const std::string &name, const BT::NodeConfiguration &config) {
                                            return std::make_unique<StopMotion>(name, cmd_vel_pub);
                                        });

    auto tree = factory.createTreeFromFile(BT_XML_PATH);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Tree loaded! Starting execution.");

    // Run through tree over and over until user stops the process or the tree fails.
    rclcpp::Rate loop_rate(10);
    while (rclcpp::ok()) {
        // Run tree once.
        // If the tree fails, then break.
        try {
            BT::NodeStatus status = tree.tickRoot();
            if (status != BT::NodeStatus::SUCCESS) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                             "Tree execution failed! Status = %d", (int) status);
                break;
            }
        } catch (BT::RuntimeError &e) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                         "Tree execution failed! Error = %s", e.what());
            break;
        }
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}