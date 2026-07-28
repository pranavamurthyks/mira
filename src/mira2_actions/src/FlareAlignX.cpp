#include "mira2_actions/AlignX.hpp"
#include "behaviortree_cpp/action_node.h"
#include "custom_msgs/msg/commands.hpp"

class FlareAlignX : public BT::StatefulActionNode
{
    FlareAlignX(const std::string& name, BT::NodeConfig& config) : BT::StatefulActionNode(name, config) 
    {

    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("x_error")
        };
    }

    

    BT::NodeStatus onStart() override
    {
        auto x_error = getInput<double>("x_error");
        if (!x_error) return BT::NodeStatus::FAILURE;

        if (std::abs(x_error.value()) > 0.10) return BT::NodeStatus::RUNNING;

        else return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus onRunning() override
    {
        auto x_error = getInput<double>("x_error");

        if (!x_error)
            return BT::NodeStatus::FAILURE;

        std::cout << "Aligning X" << std::endl;

        if (std::abs(x_error.value()) <= 0.05)
            return BT::NodeStatus::SUCCESS;

        

        return BT::NodeStatus::RUNNING;
    }
};