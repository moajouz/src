#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <fstream>
#include <sstream>

using namespace std::chrono_literals;

class HSLAMPublisher : public rclcpp::Node
{
public:
    HSLAMPublisher() : Node("hslam_publisher")
    {
        // Create publisher for H-SLAM data on topic 'hslam_data'
        publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("hslam_data", 10);

        // Load H-SLAM data from text file
        std::string txt_file_path = "/home/mooo/aub/datasets/modified/Hslam_ficosa_5_0.txt";
        loadHSLAMData(txt_file_path);

        // Publish H-SLAM data periodically with a ms delay
        timer_ = this->create_wall_timer(300ms, std::bind(&HSLAMPublisher::publishHSLAMData, this));
    }

private:
    void loadHSLAMData(const std::string& txt_file_path)
    {
        // Open and read text file
        std::ifstream file(txt_file_path);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open text file: %s", txt_file_path.c_str());
            return;
        }

        std::string line;
        // Read timestamp, x, y, z columns from text file
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            double timestamp, x, y, z;

            // Read timestamp from the 1st column
            iss >> timestamp;

            // Read x, y, z from the 2nd, 3rd, and 4th columns respectively
            iss >> x >> y >> z;

            // Store H-SLAM data
            hslam_data_vec_.emplace_back(timestamp, x, y, z);
        }

        file.close();
    }

    void publishHSLAMData()
    {  
        if (hslam_data_index_ < hslam_data_vec_.size()) {
            auto& hslam_data = hslam_data_vec_[hslam_data_index_];
            auto msg = std::make_unique<nav_msgs::msg::Odometry>();
            
            // Assigning coordinates directly
            msg->pose.pose.position.x = hslam_data.x;
            msg->pose.pose.position.y = hslam_data.y;
            msg->pose.pose.position.z = hslam_data.z;

            // Publish H-SLAM data
            publisher_->publish(std::move(msg));

            // Log message after publishing H-SLAM data
            RCLCPP_INFO(this->get_logger(), "Published H-SLAM data [%d/%d]: Timestamp=%.0f Position=%.16f, %.16f, %.15f",
                            hslam_data_index_ + 1, hslam_data_vec_.size(),
                            hslam_data.timestamp, hslam_data.x, hslam_data.y, hslam_data.z);

            // Increment index for next data point
            ++hslam_data_index_;
        } else {
            // Stop the timer if all data has been published
            timer_->cancel();
            RCLCPP_INFO(this->get_logger(), "Finished publishing all H-SLAM data.");
        }
    }

private:
    struct HSLAMData {
        double timestamp;
        double x;
        double y;
        double z;

        HSLAMData(double timestamp_, double x_, double y_, double z_)
            : timestamp(timestamp_), x(x_), y(y_), z(z_) {}
    };

    std::vector<HSLAMData> hslam_data_vec_;
    size_t hslam_data_index_ = 0;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HSLAMPublisher>());
    rclcpp::shutdown();
    return 0;
}
