#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/header.hpp"

class TimestampForwardingNode : public rclcpp::Node
{
public:
    TimestampForwardingNode() : Node("get_timestamp")
    {
        // Create subscribers for both topics
        sub_hslam_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "hslam_data", 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                this->publishTimestamp(msg->header.stamp, "hslam");
            });

        sub_gps_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "gps_data", 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                this->publishTimestamp(msg->header.stamp, "gps");
            });

        // Create publisher for timestamps
        timestamp_publisher_ = this->create_publisher<std_msgs::msg::Header>("ordered_timestamps", 10);
    }

private:
    void publishTimestamp(const builtin_interfaces::msg::Time &stamp, const std::string &frame_id)
    {
        // Create a new Header message to publish the timestamp
        auto timestamp_msg = std_msgs::msg::Header();
        timestamp_msg.stamp = stamp;
        timestamp_msg.frame_id = frame_id;

        // Publish the Header message
        timestamp_publisher_->publish(timestamp_msg);

        // Correct logging format
        RCLCPP_INFO(this->get_logger(), "Published timestamp: sec=%d, nanosec=%d, frame_id=%s",
                    timestamp_msg.stamp.sec, timestamp_msg.stamp.nanosec, timestamp_msg.frame_id.c_str());
    }

    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_hslam_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_gps_;

    // Publisher
    rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr timestamp_publisher_;
};

// Main function
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TimestampForwardingNode>());
    rclcpp::shutdown();
    return 0;
}
