#include "rclcpp/rclcpp.hpp"  // Include the main header file for ROS 2 C++ API
#include "nav_msgs/msg/odometry.hpp"  // Include the header for Odometry message type
#include <fstream>  // Include the header for file stream operations
#include <sstream>  // Include the header for string stream operations
#include <string>  // Include the header for string operations
#include <vector>  // Include the header for vector operations
#include <thread>  // Include the header for thread operations
#include <atomic>  // Include the header for atomic operations

class MixedDataPublisher : public rclcpp::Node  // Define a class that inherits from rclcpp::Node
{
public:
    MixedDataPublisher() : Node("data_publisher"), data_publishing_done_(false)  // Constructor initializes the node with the name "data_publisher" and sets the flag to false
    {
        // Create publishers for the "hslam_data" and "gps_data" topics with a queue size of 10
        hslam_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("hslam_data", 10);
        gps_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("gps_data", 10);

        // File path
        std::string merged_file_path = "/home/mooo/aub/datasets/ficosa_for_HSLAM/new_camera/Merged_results_LLA/merged_output_4_may.txt";
        
        // Load and publish data from the merged file in a separate thread
        data_thread_ = std::thread(&MixedDataPublisher::loadAndPublishData, this, merged_file_path);
    }

    ~MixedDataPublisher() {
        if (data_thread_.joinable()) {
            data_thread_.join();  // Ensure the data publishing thread is joined before destruction
        }
    }

private:
    // Method to load and publish data from a file
    void loadAndPublishData(const std::string &file_path)
    {
        std::ifstream file(file_path);  // Open the file
        if (!file.is_open()) {  // Check if the file was opened successfully
            RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", file_path.c_str());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Successfully opened file: %s", file_path.c_str());

        std::vector<std::string> lines;
        std::string line;

        // Read the file and store lines in a vector
        while (std::getline(file, line)) {
            lines.push_back(line);
        }
        file.close();  // Close the file

        // Process lines
        for (const auto& line : lines) {
            std::istringstream iss(line);  // Create a string stream for the current line
            int id;
            double timestamp, x, y, z;

            // Read the id, timestamp, and position data
            if (!(iss >> id >> timestamp >> x >> y >> z)) {
                RCLCPP_WARN(this->get_logger(), "Skipping malformed line: %s", line.c_str());
                continue;  // Skip lines that don't match the expected format
            }

            if (id == 0) {
                publishHSLAMData(timestamp, x, y, z);
            } else {
                publishGPSData(timestamp, x, y, z);
            }

            // Sleep for 10 milliseconds between processing data
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        data_publishing_done_ = true;  // Set the flag to indicate data publishing is done
    }

    // Method to publish H-SLAM data
    void publishHSLAMData(double timestamp, double x, double y, double z)
    {
        // Create a new Odometry message
        auto msg = std::make_unique<nav_msgs::msg::Odometry>();
        msg->header.stamp = this->now();
        msg->header.frame_id = "odom";  // Set the frame id to "odom"
        msg->pose.pose.position.x = x;
        msg->pose.pose.position.y = y;
        msg->pose.pose.position.z = z;

        // Publish the message
        hslam_publisher_->publish(std::move(msg));

        // Log the published H-SLAM data
        // RCLCPP_INFO(this->get_logger(), "Published H-SLAM data: ID=%.0f Position=(%.16f, %.16f, %.15f)", timestamp, x, y, z);
    }

    // Method to publish GPS data
    void publishGPSData(double timestamp, double x_gps, double y_gps, double z_gps)
    {
        // Create a new Odometry message
        auto msg = std::make_unique<nav_msgs::msg::Odometry>();
        msg->header.stamp = this->now();
        msg->header.frame_id = "gps";  // Set the frame id to "gps"
        msg->pose.pose.position.x = x_gps;
        msg->pose.pose.position.y = y_gps;
        msg->pose.pose.position.z = z_gps;

        // Publish the message
        gps_publisher_->publish(std::move(msg));

        // Log the published GPS data
        // RCLCPP_INFO(this->get_logger(), "Published GPS data: ID=%.0f Position=(%.16f, %.16f, %.15f)", timestamp, x_gps, y_gps, z_gps);
    }

    // Publishers for H-SLAM and GPS data
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr hslam_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr gps_publisher_;

    // Thread for publishing data
    std::thread data_thread_;

    // Flag to indicate if data publishing is done
    std::atomic<bool> data_publishing_done_;
};

// Main function to initialize the ROS 2 system and run the node
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);  // Initialize the ROS 2 system

    rclcpp::spin(std::make_shared<MixedDataPublisher>());  // Create and run the node

    rclcpp::shutdown();  // Shutdown the ROS 2 system
    return 0;  // Return success status
}
