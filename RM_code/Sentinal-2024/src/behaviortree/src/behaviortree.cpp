#include "behaviortree/behaviortree.h"

#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>
#include <std_msgs/msg/detail/float32_multi_array__struct.hpp>
#include <std_msgs/msg/detail/int32_multi_array__struct.hpp>
#include <vector>

//gIntDataInit
gIntDataInit::gIntDataInit(
    const std::string& name,
    const BT::NodeConfig& conf,
    const BT::RosNodeParams& params)
    : BT::RosTopicSubNode<std_msgs::msg::Int32MultiArray>(name, conf, params) {
    std::cout << "gIntDataInit works!!!" << std::endl;
}

BT::PortsList gIntDataInit::providedPorts() {
    BT::PortsList ports_list;
    ports_list.insert(BT::OutputPort<int>("game_time"));
    ports_list.insert(BT::OutputPort<int>("game_economy"));
    ports_list.insert(BT::OutputPort<int>("state_outpost"));
    ports_list.insert(BT::OutputPort<int>("state_guard"));
    ports_list.insert(BT::OutputPort<int>("state_base"));
    ports_list.insert(BT::OutputPort<int>("state_purchase"));
    ports_list.insert(BT::OutputPort<int>("remaining_ammunition"));
    return ports_list;
}

BT::NodeStatus gIntDataInit::onTick(const std::shared_ptr<std_msgs::msg::Int32MultiArray>& msg) {
    if (msg == NULL) {
        return BT::NodeStatus::FAILURE;
    }
    game_time = (msg->data)[0];
    game_economy = (msg->data)[1];
    state_outpost = (msg->data)[2];
    state_guard = (msg->data)[3];
    state_base = (msg->data)[4];
    state_purchase = (msg->data)[5];
    remaining_ammunition = (msg->data)[6];
    setOutput("game_time", game_time);
    setOutput("game_economy", game_economy);
    setOutput("state_outpost", state_outpost);
    setOutput("state_guard", state_guard);
    setOutput("state_base", state_base);
    setOutput("state_purchase", state_purchase);
    setOutput("remaining_ammunition", remaining_ammunition);
    return BT::NodeStatus::SUCCESS;
}

//alFloatDataInit
alFloatDataInit::alFloatDataInit(
    const std::string& name,
    const BT::NodeConfig& conf,
    const BT::RosNodeParams& params)
    : BT::RosTopicSubNode<std_msgs::msg::Float32MultiArray>(name, conf, params) {
    std::cout << "alFloatDataInit works!!!" << std::endl;
}

BT::PortsList alFloatDataInit::providedPorts() {
    BT::PortsList ports_list;
    ports_list.insert(BT::OutputPort<float>("artificial_location_x"));
    ports_list.insert(BT::OutputPort<float>("artificial_location_y"));
    return ports_list;
}

BT::NodeStatus alFloatDataInit::onTick(const std::shared_ptr<std_msgs::msg::Float32MultiArray>& msg) {
    if (msg == NULL) {
        return BT::NodeStatus::FAILURE;
    }
    artificial_location_x = (msg->data)[0];

    artificial_location_y = (msg->data)[1];
    setOutput("artificial_location_x", artificial_location_x);
    setOutput("artificial_location_y", artificial_location_y);
    return BT::NodeStatus::SUCCESS;
}

//pFloatDataInit
pFloatDataInit::pFloatDataInit(
    const std::string& name,
    const BT::NodeConfig& conf,
    const BT::RosNodeParams& params)
    : BT::RosTopicSubNode<std_msgs::msg::Float32MultiArray>(name, conf, params) {
    std::cout << "pFloatDataInit works!!!" << std::endl;
}

BT::PortsList pFloatDataInit::providedPorts() {
    BT::PortsList ports_list;
    ports_list.insert(BT::OutputPort<float>("position_x"));
    ports_list.insert(BT::OutputPort<float>("position_y"));
    return ports_list;
}

BT::NodeStatus pFloatDataInit::onTick(const std::shared_ptr<std_msgs::msg::Float32MultiArray>& msg) {
    if (msg == NULL) {
        return BT::NodeStatus::FAILURE;
    }
    position_x = (msg->data)[0];
    position_y = (msg->data)[1];
    setOutput("position_x", position_x);
    setOutput("position_y", position_y);
    return BT::NodeStatus::SUCCESS;
}

//autoResurrection
autoResurrection::autoResurrection(
    const std::string& name,
    const BT::NodeConfig& conf,
    const BT::RosNodeParams& params)
    : BT::RosTopicPubNode<std_msgs::msg::Int32MultiArray>(name, conf, params) {
    std::cout << "autoResurrection works!!!" << std::endl;
}

BT::PortsList autoResurrection::providedPorts() {
    BT::PortsList ports_list;
    ports_list.insert(BT::InputPort<int>("game_economy"));
    ports_list.insert(BT::InputPort<int>("game_time"));
    ports_list.insert(BT::InputPort<int>("state_guard"));
    return ports_list;
}

bool autoResurrection::setMessage(std_msgs::msg::Int32MultiArray& msg) {
    auto game_economy = getInput<int>("game_economy");
    if (!game_economy) {
        throw BT::RuntimeError("missing required input [game_economy]: ", game_economy.error());
    }
    auto game_time = getInput<int>("game_time");
    if (!game_time) {
        throw BT::RuntimeError("missing required input [game_time]: ", game_time.error());
    }
    auto state_guard = getInput<int>("state_guard");
    if (!state_guard) {
        throw BT::RuntimeError("missing required input [state_guard]: ", state_guard.error());
    }
    std::vector<int> data;
    if (state_guard.value() == 0 && game_economy.value() > 500 && game_time.value() > 300) {
        data.push_back(1);
        data.push_back(1);
    } else {
        data.push_back(0);
        data.push_back(0);
    }
    msg.data = data;
    return true;
}

//autoPurchase
autoPurchase::autoPurchase(
    const std::string& name,
    const BT::NodeConfig& conf,
    const BT::RosNodeParams& params)
    : BT::RosTopicPubNode<std_msgs::msg::Int32MultiArray>(name, conf, params) {
    std::cout << "autoPurchase works!!!" << std::endl;
}

BT::PortsList autoPurchase::providedPorts() {
    BT::PortsList ports_list;
    ports_list.insert(BT::InputPort<int>("game_economy"));
    ports_list.insert(BT::InputPort<int>("remaining_ammunition"));
    return ports_list;
}

bool autoPurchase::setMessage(std_msgs::msg::Int32MultiArray& msg) {
    auto game_economy = getInput<int>("game_economy");
    if (!game_economy) {
        throw BT::RuntimeError("missing required input [game_economy]: ", game_economy.error());
    }
    auto remaining_ammunition = getInput<int>("remaining_ammunition");
    if (!remaining_ammunition) {
        throw BT::RuntimeError(
            "missing required input [remaining_ammunition]: ",
            remaining_ammunition.error());
    }
    std::vector<int> data;
    if (game_economy.value() > 300 && remaining_ammunition.value() < 50) {
        data.push_back(2);
        data.push_back(50);
    } 
    else {
        data.push_back(0);
        data.push_back(0);
    }
    msg.data = data;
    return true;
}

//stateOutpostJudgement
stateOutpostJudgement::stateOutpostJudgement(const std::string& name, const BT::NodeConfig& conf)
    : BT::SyncActionNode(name, conf) {
    std::cout << "stateOutpostJudgement works!!!" << std::endl;
}

BT::PortsList stateOutpostJudgement::providedPorts() {
    BT::PortsList ports_list;
    ports_list.insert(BT::InputPort<int>("state_outpost"));
    return ports_list;
}

BT::NodeStatus stateOutpostJudgement::tick() {
    auto state_outpost = getInput<int>("state_outpost");
    if (!state_outpost) {
        throw BT::RuntimeError("missing required input [state_outpost]: ", state_outpost.error());
    }
    if (state_outpost.value() > 0) {
        return BT::NodeStatus::SUCCESS;
    } else {
        return BT::NodeStatus::FAILURE;
    }
}

//rotateArmor
rotateArmor::rotateArmor(
    const std::string& name,
    const BT::NodeConfig& conf,
    const BT::RosNodeParams& params)
    : BT::RosTopicPubNode<std_msgs::msg::Int32MultiArray>(name, conf, params) {
    std::cout << "rotateArmor works!!!" << std::endl;
}

bool rotateArmor::setMessage(std_msgs::msg::Int32MultiArray& msg) {
    std::vector<int> data;
    data.push_back(1);
    data.push_back(1);
    msg.data = data;
    return true;
}

//purchaseJudgement
purchaseJudgement::purchaseJudgement(const std::string& name, const BT::NodeConfig& conf)
    : BT::SyncActionNode(name, conf) {
    std::cout << "purchaseJudgement works!!!" << std::endl;
}

BT::PortsList purchaseJudgement::providedPorts() {
    BT::PortsList ports_list;
    ports_list.insert(BT::InputPort<int>("state_purchase"));
    return ports_list;
}

BT::NodeStatus purchaseJudgement::tick() {
    auto state_purchase = getInput<int>("state_purchase");
    if (!state_purchase) {
        throw BT::RuntimeError("missing required input [state_purchase]: ", state_purchase.error());
    }
    if (state_purchase.value() == 1) {
        return BT::NodeStatus::SUCCESS;
    } else {
        return BT::NodeStatus::FAILURE;
    }
}

//moveTo
moveTo::moveTo(
    const std::string& name,
    const BT::NodeConfig& conf,
    const BT::RosNodeParams& params)
    : BT::RosTopicPubNode<std_msgs::msg::Float32MultiArray>(name, conf, params) {
    std::cout << "moveTo works!!!" << std::endl;
}

BT::PortsList moveTo::providedPorts() {
    BT::PortsList ports_list;
    ports_list.insert(BT::InputPort<float>("destination_location_x"));
    ports_list.insert(BT::InputPort<float>("destination_location_y"));
    return ports_list;
}

bool moveTo::setMessage(std_msgs::msg::Float32MultiArray& msg) {
    auto destination_location_x = getInput<float>("destination_location_x");
    if (!destination_location_x) {
        throw BT::RuntimeError(
            "missing required input [destination_location_x]: ",
            destination_location_x.error());
    }
    auto destination_location_y = getInput<float>("destination_location_y");
    if (!destination_location_y) {
        throw BT::RuntimeError(
            "missing required input [destination_location_y]: ",
            destination_location_y.error());
    }
    std::vector<float> data;
    data.push_back(destination_location_x.value());
    data.push_back(destination_location_y.value());
    msg.data = data;
    return true;
}

//guardJudgement
guardJudgement::guardJudgement(const std::string& name, const BT::NodeConfig& conf)
    : BT::SyncActionNode(name, conf) {
    std::cout << "guardJudgement works!!!" << std::endl;
}

BT::PortsList guardJudgement::providedPorts() {
    BT::PortsList ports_list;
    ports_list.insert(BT::InputPort<int>("state_guard"));
    return ports_list;
}

BT::NodeStatus guardJudgement::tick() {
    auto state_guard = getInput<int>("state_guard");
    if (!state_guard) {
        throw BT::RuntimeError("missing required input [state_guard]: ", state_guard.error());
    }
    if (state_guard.value() > 150) {
        return BT::NodeStatus::SUCCESS;
    } else {
        return BT::NodeStatus::FAILURE;
    }
}

//artificialControlJudgement
artificialControlJudgement::artificialControlJudgement(
    const std::string& name,
    const BT::NodeConfig& conf)
    : BT::SyncActionNode(name, conf) {
    std::cout << "artificialControlJudgement works!!!" << std::endl;
}

BT::PortsList artificialControlJudgement::providedPorts() {
    BT::PortsList ports_list;
    ports_list.insert(BT::InputPort<float>("artificial_location_x"));
    ports_list.insert(BT::InputPort<float>("artificial_location_y"));
    return ports_list;
}

BT::NodeStatus artificialControlJudgement::tick() {
    auto artificial_location_x = getInput<float>("artificial_location_x");
    if (!artificial_location_x) {
        throw BT::RuntimeError(
            "missing required input [artificial_location_x]: ",
            artificial_location_x.error());
    }
    auto artificial_location_y = getInput<float>("artificial_location_y");
    if (!artificial_location_y) {
        throw BT::RuntimeError(
            "missing required input [artificial_location_y]: ",
            artificial_location_y.error());
    }
    if (artificial_location_x.value() - 0.0 < 0.1 && artificial_location_y.value() - 0.0 < 0.1) {
        return BT::NodeStatus::SUCCESS;
    } else {
        return BT::NodeStatus::FAILURE;
    }
}

//gameTimeJudgement
gameTimeJudgement::gameTimeJudgement(const std::string& name, const BT::NodeConfig& conf)
    : BT::SyncActionNode(name, conf) {
    std::cout << "gameTimeJudgement works!!!" << std::endl;
}

BT::PortsList gameTimeJudgement::providedPorts() {
    BT::PortsList ports_list;
    ports_list.insert(BT::InputPort<int>("before_time"));
    ports_list.insert(BT::InputPort<int>("game_time"));
    ports_list.insert(BT::InputPort<int>("late_time"));
    return ports_list;
}

BT::NodeStatus gameTimeJudgement::tick() {
    auto before_time = getInput<int>("before_time");
    if (!before_time) {
        throw BT::RuntimeError("missing required input [before_time]: ", before_time.error());
    }
    auto game_time = getInput<int>("game_time");
    if (!game_time) {
        throw BT::RuntimeError("missing required input [game_time]: ", game_time.error());
    }
    auto late_time = getInput<int>("late_time");
    if (!late_time) {
        throw BT::RuntimeError("missing required input [late_time]: ", late_time.error());
    }
    if (game_time.value() >= before_time.value() && game_time.value() <= late_time.value()) {
        return BT::NodeStatus::FAILURE;
    } else {
        return BT::NodeStatus::SUCCESS;
    }
}