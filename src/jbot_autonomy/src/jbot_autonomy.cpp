#include "rclcpp/rclcpp.hpp"
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

StopMotion::StopMotion(const std::string &name)
        : SyncActionNode(name, {}) {}

BT::NodeStatus StopMotion::tick() {
    // TODO actually publish
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Publishing 'zero' Twist on /cmd_vel");
    return BT::NodeStatus::SUCCESS;
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<SetBlackboardInputs>("SetBlackboardInputs");
    factory.registerNodeType<IsBotModeIdle>("IsBotModeIdle");
    factory.registerNodeType<StopMotion>("StopMotion");

    auto tree = factory.createTreeFromFile(BT_XML_PATH);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Tree loaded! Starting execution.");

    rclcpp::Rate loop_rate(10);
    // Run through tree over and over until user stops the process
    while (rclcpp::ok()) {
        // Run tree once
        BT::NodeStatus status = tree.tickRoot();
        if (status != BT::NodeStatus::SUCCESS) {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                        "Tree execution failed!. Status = %i", status);
            break;
        }
        loop_rate.sleep();
    }

    return 0;
}