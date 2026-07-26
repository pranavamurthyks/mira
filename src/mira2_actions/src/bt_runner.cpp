#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.h"



class BTRunner : public rclcpp::Node
{
    public:
    BTRunner() : rclcpp::Node("bt_runner")
    {
        orangeFlareSubscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>("/orange_flare/pose", 10, callback_function);
    }

    void flare_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        
    }


    // Change the variable for isflarevisible 

    private:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr orangeFlareSubscriber;



};

// Main to keep the node running 