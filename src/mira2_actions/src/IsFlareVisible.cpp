#include "mira2_actions/IsFlareVisible.hpp"

IsFlareVisible::IsFlareVisible(
    const std::string& name,
    const BT::NodeConfig& config
)
    : BT::ConditionNode(name, config)
{
}

BT::NodeStatus IsFlareVisible::tick()
{
    auto flare_pose_z = getInput<double>("flare_z");

    if (!flare_pose_z)
        return BT::NodeStatus::FAILURE;

    if (flare_pose_z.value() != -1.0)
        return BT::NodeStatus::SUCCESS;

    return BT::NodeStatus::FAILURE;
}