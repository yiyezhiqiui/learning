#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/blackboard.h>
#include <behaviortree_cpp/bt_factory.h>
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/utilities.hpp>

#include "behaviortree/behaviortree.h"
#include "behaviortree_ros2/ros_node_params.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto gdata_nh = std::make_shared<rclcpp::Node>("gdata_node");
    auto aldata_nh = std::make_shared<rclcpp::Node>("aldata_node");
    auto pdata_nh = std::make_shared<rclcpp::Node>("pdata_node");
    auto ar_nh = std::make_shared<rclcpp::Node>("ar_node");
    auto ap_nh = std::make_shared<rclcpp::Node>("ap_node");
    auto ra_nh = std::make_shared<rclcpp::Node>("ra_node");
    auto mt_nh = std::make_shared<rclcpp::Node>("mt_node");

    BT::BehaviorTreeFactory factory;

    BT::RosNodeParams gdata_params, aldata_params, pdata_params, ar_params, ap_params, ra_params, mt_params;

    //订阅者
    gdata_params.nh = gdata_nh;
    gdata_params.default_port_value = "communicate/gameinfo";
    aldata_params.nh = aldata_nh;
    aldata_params.default_port_value = "communicate/artificial/location";
    pdata_params.nh = pdata_nh;
    pdata_params.default_port_value = "communicate/position";

    //发布者
    ar_params.nh = ar_nh;
    ar_params.default_port_value = "behaviortree/interaction";
    ap_params.nh = ap_nh;
    ap_params.default_port_value = "behaviortree/interaction";
    ra_params.nh = ar_nh;
    ra_params.default_port_value = "behaviortree/moudle";
    mt_params.nh = mt_nh;
    mt_params.default_port_value = "behaviortree/move";

    //订阅者
    factory.registerNodeType<gIntDataInit>("gIntDataInit", gdata_params);
    factory.registerNodeType<alFloatDataInit>("alFloatDataInit", aldata_params);
    factory.registerNodeType<pFloatDataInit>("pFloatDataInit", pdata_params);

    //发布者
    factory.registerNodeType<autoResurrection>("autoResurrection", ar_params);
    factory.registerNodeType<autoPurchase>("autoPurchase", ap_params);
    factory.registerNodeType<stateOutpostJudgement>("stateOutpostJudgement");
    factory.registerNodeType<rotateArmor>("rotateArmor", ra_params);
    factory.registerNodeType<purchaseJudgement>("purchaseJudgement");
    factory.registerNodeType<moveTo>("moveTo", mt_params);

    //无ros2的节点
    factory.registerNodeType<guardJudgement>("guardJudgement");
    factory.registerNodeType<artificialControlJudgement>("artificialControlJudgement");
    factory.registerNodeType<gameTimeJudgement>("gameTimeJudgement");

    auto blackboard = BT::Blackboard::create();
    auto tree = factory.createTreeFromFile("src/behaviortree/src/mytree.xml", blackboard);
    while (rclcpp::ok()) {
        tree.tickWhileRunning();
    }
    return 0;
}
/*BTCPP_format="4"*/
