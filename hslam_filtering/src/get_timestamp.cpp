#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/header.hpp"
#include <queue>
#include <memory>

class TimestampOrderingNode : public rclcpp::Node
{
public:
    TimestampOrderingNode() : Node("timestamp_ordering_node")
    {
        // Create subscribers for both topics
        sub_hslam_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "hslam_data", 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                this->hslam_callback(msg);
            });

        sub_gps_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "gps_data", 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                this->gps_callback(msg);
            });

        // Create publisher for ordered timestamps
        timestamp_publisher_ = this->create_publisher<std_msgs::msg::Header>("ordered_timestamps", 10);
    }

private:
    // Callback functions for subscribers
    void hslam_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Store the H-SLAM timestamp in the priority queue
        timestamps_.emplace(msg->header.stamp, "hslam");
        process_timestamps();
    }

    void gps_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Store the GPS timestamp in the priority queue
        timestamps_.emplace(msg->header.stamp, "gps");
        process_timestamps();
    }

    void process_timestamps()
    {
        while (!timestamps_.empty())
        {
            // Get the earliest timestamp
            auto timestamp = timestamps_.top();
            timestamps_.pop();

            // Create a new Header message to publish the ordered timestamp
            auto timestamp_msg = std_msgs::msg::Header();
            timestamp_msg.stamp = timestamp.first;
            timestamp_msg.frame_id = timestamp.second;

            // Publish the Header message
            timestamp_publisher_->publish(timestamp_msg);
            RCLCPP_INFO(this->get_logger(), "Published timestamp: sec: %d, nanosec: %d, frame_id: %s",
                        timestamp_msg.stamp.sec, timestamp_msg.stamp.nanosec, timestamp_msg.frame_id.c_str());
        }
    }

    // Priority queue to store timestamps and their origin
    using TimestampEntry = std::pair<builtin_interfaces::msg::Time, std::string>;
    struct TimestampComparator
    {
        bool operator()(const TimestampEntry &a, const TimestampEntry &b) const
        {
            // Compare timestamps; the priority queue uses this for ordering
            return (a.first.sec * 1e9 + a.first.nanosec) > (b.first.sec * 1e9 + b.first.nanosec);
        }
    };
    std::priority_queue<TimestampEntry, std::vector<TimestampEntry>, TimestampComparator> timestamps_;

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
    rclcpp::spin(std::make_shared<TimestampOrderingNode>());
    rclcpp::shutdown();
    return 0;
}
