#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::placeholders;

class subscriberNode : public rclcpp::Node
{
public:
    subscriberNode() : Node("subscriber_node")
    {
        subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "temperature", 1,
            std::bind(&subscriberNode::getTemperature, this, _1)
        );
        RCLCPP_INFO(this->get_logger(), "subscriber Node has been started");
    }

private:
    void getTemperature(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received temperature: '%s'", msg->data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<subscriberNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}