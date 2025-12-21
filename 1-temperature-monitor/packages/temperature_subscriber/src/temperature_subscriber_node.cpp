#include "rclcpp/rclcpp.hpp"
#include "temperature_monitoring_interfaces/msg/temperature_data.hpp"
#include <deque>
#include <algorithm>
#include <numeric>
#include <string>

using namespace std::placeholders;

class subscriberNode : public rclcpp::Node
{
public:
    subscriberNode() : Node("temperature_subscriber_node")
    {
        static const rmw_qos_profile_t qos_services = {
            RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            1,  // message queue depth
            RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            RMW_QOS_DEADLINE_DEFAULT,
            RMW_QOS_LIFESPAN_DEFAULT,
            RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
            RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
            false};

        subscriber_ = this->create_subscription<temperature_monitoring_interfaces::msg::TemperatureData>(
            "temperature",
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_services), qos_services),
            std::bind(&subscriberNode::getTemperature, this, _1)
        );
        RCLCPP_INFO(this->get_logger(), "subscriber Node has been started");
    }

private:
    void getTemperature(const temperature_monitoring_interfaces::msg::TemperatureData::SharedPtr msg)
    {
        double temp = msg->temperature;
        std::string unit = msg->unit;
        std::string sensor_id = msg->sensor_id;

        // Store last 10 readings
        if (recent_temps_.size() == 10)
            recent_temps_.pop_front();
        recent_temps_.push_back(temp);

        // Update min/max
        if (recent_temps_.size() == 1) {
            min_temp_ = max_temp_ = temp;
        } else {
            min_temp_ = std::min(min_temp_, temp);
            max_temp_ = std::max(max_temp_, temp);
        }

        // Calculate moving average
        double avg = std::accumulate(recent_temps_.begin(), recent_temps_.end(), 0.0) / recent_temps_.size();

        // Determine trend
        std::string trend = "stable";
        if (recent_temps_.size() >= 2) {
            double diff = recent_temps_.back() - *(recent_temps_.rbegin() + 1);
            if (std::abs(diff) < 0.01) {
                trend = "stable";
            } else if (diff > 0) {
                trend = "rising";
            } else {
                trend = "falling";
            }
        }

        // Warning system
        const double NORMAL_MIN = 23.0;
        const double NORMAL_MAX = 27.0; 
        if (temp < NORMAL_MIN || temp > NORMAL_MAX) {
            RCLCPP_WARN(this->get_logger(), "WARNING: Temperature %.2f %s from sensor '%s' is OUTSIDE normal range!", temp, unit.c_str(), sensor_id.c_str());
        }

        // Display info
        RCLCPP_INFO(this->get_logger(),
            "Received temperature: %.2f %s from sensor '%s' | Moving Avg: %.2f | Min: %.2f | Max: %.2f | Trend: %s",
            temp, unit.c_str(), sensor_id.c_str(), avg, min_temp_, max_temp_, trend.c_str());
    }

    rclcpp::Subscription<temperature_monitoring_interfaces::msg::TemperatureData>::SharedPtr subscriber_;
    std::deque<double> recent_temps_;
    double min_temp_ = 0.0;
    double max_temp_ = 0.0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<subscriberNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}