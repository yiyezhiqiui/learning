#include "node/condition/is_status_ok.hpp"

namespace behaviortree
{
    IsStatusOk::IsStatusOk(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SimpleConditionNode(name, std::bind(IsStatusOk::CheckSentryStatus, this), config)
    {
    }

    BT::PortsList IsStatusOk::providedPorts()
    {
        BT::PortsList ports_list;
        ports_list.insert(BT::InputPort<behaviortree_interfaces::msg::SentryStatus>("sentry_status"));
        ports_list.insert(BT::InputPort<int>("sentry_hp_threshold"));
        ports_list.insert(BT::InputPort<int>("sentry_heat_threshold"));
        return ports_list;
    }

    BT::NodeStatus IsStatusOk::CheckSentryStatus()
    {
        auto sentry_status = getInput<behaviortree_interfaces::msg::SentryStatus>("sentry_status");
        auto sentry_hp_threshold = getInput<int>("sentry_hp_threshold");
        auto sentry_heat_threshold = getInput<int>("sentry_heat_threshold");
        if (!sentry_status)
        {
            throw BT::RuntimeError("missing required input [sentry_status]: ", sentry_status.error());
            return BT::NodeStatus::FAILURE;
        }
        if (!sentry_hp_threshold)
        {
            throw BT::RuntimeError("missing required input [sentry_hp_threshold]: ", sentry_hp_threshold.error());
            return BT::NodeStatus::FAILURE;
        }
        if (!sentry_heat_threshold)
        {
            throw BT::RuntimeError("missing required input [sentry_heat_threshold]: ", sentry_heat_threshold.error());
            return BT::NodeStatus::FAILURE;
        }

        if(sentry_status->current_hp<=sentry_hp_threshold.value()||sentry_status->shooter_heat>=sentry_heat_threshold.value())
        {
            return BT::NodeStatus::FAILURE;
        }
        else
        {
            return BT::NodeStatus::SUCCESS;
        }
    }
}
