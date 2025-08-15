#include "rclcpp/rclcpp.hpp"
#include "temperature_monitoring_interfaces/msg/temperature_data.hpp"

using namespace std::placeholders;

class subscriberNode : public rclcpp::Node
{
public:
    subscriberNode() : Node("subscriber_node")
    {
        subscriber_ = this->create_subscription<temperature_monitoring_interfaces::msg::TemperatureData>(
            "temperature", 10,
            std::bind(&subscriberNode::getTemperature, this, _1)
        );
        RCLCPP_INFO(this->get_logger(), "subscriber Node has been started");
    }

private:
    void getTemperature(const temperature_monitoring_interfaces::msg::TemperatureData::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received temperature: '%f %s' from sensor '%s'",
                    msg->temperature, msg->unit.c_str(), msg->sensor_id.c_str());
    }
    rclcpp::Subscription<temperature_monitoring_interfaces::msg::TemperatureData>::SharedPtr subscriber_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<subscriberNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}