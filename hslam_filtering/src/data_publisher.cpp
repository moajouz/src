#include "rclcpp/rclcpp.hpp"  // Include the main header file for ROS 2 C++ API
#include "nav_msgs/msg/odometry.hpp"  // Include the header for Odometry message type
#include "builtin_interfaces/msg/time.hpp"
#include <fstream>  // Include the header for file stream operations
#include <sstream>  // Include the header for string stream operations
#include <string>  // Include the header for string operations
#include <vector>  // Include the header for vector operations
#include <thread>  // Include the header for thread operations
#include <atomic>  // Include the header for atomic operations

class MixedDataPublisher : public rclcpp::Node  // Define a class that inherits from rclcpp::Node
{
public:
    MixedDataPublisher() : Node("data_publisher"), data_publishing_done_(false)  // Constructor initializes the node with the name "mixed_data_publisher" and sets the flag to false
    {
        int file_index = 5;  // Variable for the index

        // Create publishers for the "hslam_data" and "gps_data" topics with a queue size of 10
        hslam_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("hslam_data", 10);
        gps_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("gps_data", 10);

        //initialize file path
        std::string file_path_template = "/home/mooo/aub/datasets/ficosa_for_HSLAM/Merged_results_GPS_xyz/merged_output_%d.txt";
        
        // Create the final file path using the variable
        char file_path[256];
        snprintf(file_path, sizeof(file_path), file_path_template.c_str(), file_index);
        
        //set file path
        std::string merged_file_path = file_path;
        
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

        std::vector<std::string> lines;
        std::string line;

        // Read the file and store lines in a vector
        while (std::getline(file, line)) {
            lines.push_back(line);
        }
        file.close();  // Close the file

        // Process lines
        for (size_t i = 0; i < lines.size(); ++i) {
            std::istringstream iss(lines[i]);  // Create a string stream for the current line
            int id;
            double timestamp;

            // Read the id and timestamp
            iss >> id >> timestamp;

            if (id == 0) {
                // Read x, y, z for H-SLAM data
                double x, y, z;
                iss >> x >> y >> z;

                // Check if the next line exists and is GPS data
                if (i + 1 < lines.size()) {
                    std::istringstream next_iss(lines[i + 1]);
                    int next_id;
                    double next_timestamp;
                    next_iss >> next_id >> next_timestamp;

                    if (next_id == 1) {
                        // Read the GPS data from the next line
                        double x_gps, y_gps, z_gps;
                        next_iss >> x_gps >> y_gps >> z_gps;

                        // Publish both H-SLAM and GPS data
                        publishHSLAMData(timestamp, x, y, z);
                        publishGPSData(next_timestamp, x_gps, y_gps, z_gps);
                        
                        // Skip the next line since it has been processed
                        ++i;
                    }
                }
            }

            // Sleep for 10 milliseconds between processing data
            std::this_thread::sleep_for(std::chrono::milliseconds(10));  // 100 milliseconds
        }

        data_publishing_done_ = true;  // Set the flag to indicate data publishing is done
    }

    // Method to convert Unix nanosecond timestamp to rclcpp::Time
    rclcpp::Time convertToRclcppTime(int64_t unix_nanoseconds) {
    // Extract seconds and nanoseconds from the Unix nanosecond timestamp
    int64_t seconds = unix_nanoseconds / 1e9;
    int64_t nanoseconds = unix_nanoseconds % static_cast<int64_t>(1e9);

    // Create rclcpp::Time object using seconds and nanoseconds
    return rclcpp::Time(seconds, nanoseconds);
    }


    // Method to publish H-SLAM data
    void publishHSLAMData(double timestamp, double x, double y, double z)
    {
        // Create a new Odometry message
        auto msg = std::make_unique<nav_msgs::msg::Odometry>();
        msg->header.stamp = convertToRclcppTime(timestamp);
        msg->header.frame_id = "odom";  // Set the frame id to "odom"
        msg->pose.pose.position.x = x;
        msg->pose.pose.position.y = y;
        msg->pose.pose.position.z = z;

        // Publish the message
        hslam_publisher_->publish(std::move(msg));

        // Log the published H-SLAM data
        RCLCPP_INFO(this->get_logger(), "Published H-SLAM data: %.0f %.16f %.16f %.15f", timestamp, x, y, z);
    }

    // Method to publish GPS data
    void publishGPSData(double timestamp, double x_gps, double y_gps, double z_gps)
    {
        // Create a new Odometry message
        auto msg = std::make_unique<nav_msgs::msg::Odometry>();
        msg->header.stamp = convertToRclcppTime(timestamp);
        msg->header.frame_id = "gps";  // Set the frame id to "gps"
        msg->pose.pose.position.x = x_gps;
        msg->pose.pose.position.y = y_gps;
        msg->pose.pose.position.z = z_gps;

        // Publish the message
        gps_publisher_->publish(std::move(msg));

        // Log the published GPS data
        RCLCPP_INFO(this->get_logger(), "Published GPS data: %.0f %.16f %.16f %.15f", timestamp, x_gps, y_gps, z_gps);
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
