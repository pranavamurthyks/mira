#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#include "behaviortree_cpp/blackboard.h"
#include "behaviortree_cpp/bt_factory.h"

#include "mira2_actions/IsFlareVisible.hpp"

#include <chrono>
#include <functional>

class BTRunner : public rclcpp::Node {
private:
  BT::Blackboard::Ptr blackboard;

  BT::BehaviorTreeFactory factory;
  BT::Tree tree;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      orangeFlareSubscriber;

  rclcpp::TimerBase::SharedPtr btTimer;

  bool perceptionReady = false;

public:
  BTRunner() : rclcpp::Node("bt_runner") {
    // Create blackboard
    blackboard = BT::Blackboard::create();

    // Give initial values
    blackboard->set("flare_x", 0.0);
    blackboard->set("flare_y", 0.0);
    blackboard->set("flare_z", -1.0);

    // Subscriber
    orangeFlareSubscriber =
        this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/flare/pose", 10,
            std::bind(&BTRunner::flare_callback, this, std::placeholders::_1));


custom_msgs::msg::Commands msg;

    // Register BT nodes
    factory.registerNodeType<IsFlareVisible>("IsFlareVisible");

    // Create tree
    tree = factory.createTreeFromFile(
        "/home/pranavamurthy-ks/mira/mira/src/mira2_actions/orange_flare.xml",
        blackboard);

    // Tick BT every 100 ms
    btTimer = this->create_wall_timer(std::chrono::milliseconds(100),
                                      std::bind(&BTRunner::tick_tree, this));
  }

  void flare_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    blackboard->set("flare_x", msg->pose.position.x);
    blackboard->set("flare_y", msg->pose.position.y);
    blackboard->set("flare_z", msg->pose.position.z);

    this->perceptionReady = true;
  }

  void tick_tree() {
    if (!perceptionReady)
      return;
    auto status = tree.tickOnce();

    RCLCPP_INFO(this->get_logger(), "BT status: %s", BT::toStr(status).c_str());
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<BTRunner>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}