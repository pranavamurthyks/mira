#include <iostream>
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/condition_node.h"

class IsFlareVisible : public BT::ConditionNode
{   
    public:
    IsFlareVisible(const std::string &name) : BT::ConditionNode(name, {})
    {}

    BT::NodeStatus tick() override
    {
        
    }

};