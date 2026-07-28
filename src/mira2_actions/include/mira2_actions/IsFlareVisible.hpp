#pragma once

#include "behaviortree_cpp/condition_node.h"

class IsFlareVisible : public BT::ConditionNode
{
public:
    IsFlareVisible(
        const std::string& name,
        const BT::NodeConfig& config
    );

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("flare_z")
        };
    }


    BT::NodeStatus tick() override;
};