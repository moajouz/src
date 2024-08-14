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
    MixedDataPublisher() : Node("mixed_data_publisher"), data_publishing_done_(false)  // Constructor initializes the node with the name "mixed_data_publisher" and sets the flag to false
    {
        // Create publishers for the "hslam_data" and "gps_data" topics with a queue size of 10
        hslam_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("hslam_data", 10);
        gps_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("gps_data", 10);

        // Define the path to the merged data file
        std::string merged_file_path = "/home/mooo/aub/datasets/ficosa_for_HSLAM/old_camera/Scale_B/Merged_results_GPS_xyz_my_scale_and_transformation/merged_output_5_my_scale_and_transformation.txt";
        
        // Load and publish data from the merged file in a separate thread
        data_thread_ = std::thread(&MixedDataPublisher::loadAndPublishData, this, merged_file_path);

        // Create a subscriber to the /odometry/filtered topic
        ekf_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10,
            std::bind(&MixedDataPublisher::ekfCallback, this, std::placeholders::_1)
        );

        // Open the file to save EKF data
        ekf_data_file_.open("/home/mooo/aub/datasets/ficosa_for_HSLAM/old_camera/Scale_B/Results_EKF_GPS_xyz_my_scale_and_transformation/ekf_result_5.txt", std::ios::out | std::ios::app);
        if (!ekf_data_file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open EKF data file.");
        }
        // // Create publishers for the "hslam_data" and "gps_data" topics with a queue size of 10
        // hslam_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("hslam_data", 10);
        // gps_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("gps_data", 10);

        // // Define the path to the merged data file
        // std::string merged_file_path = "/home/mooo/aub/datasets/ficosa_for_HSLAM/new_camera/Transformed_manual_corrolated/merged_output_3_may.txt";
        
        // // Load and publish data from the merged file in a separate thread
        // data_thread_ = std::thread(&MixedDataPublisher::loadAndPublishData, this, merged_file_path);

        // // Create a subscriber to the /odometry/filtered topic
        // ekf_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        //     "/odometry/filtered", 10,
        //     std::bind(&MixedDataPublisher::ekfCallback, this, std::placeholders::_1)
        // );

        // // Open the file to save EKF data
        // ekf_data_file_.open("/home/mooo/aub/datasets/ficosa_for_HSLAM/new_camera/EKF_Transformed_manual_corrolated/ekf_result_3_may.txt", std::ios::out | std::ios::app);
        // if (!ekf_data_file_.is_open()) {
        //     RCLCPP_ERROR(this->get_logger(), "Failed to open EKF data file.");
        // }
    }

    ~MixedDataPublisher() {
        if (data_thread_.joinable()) {
            data_thread_.join();  // Ensure the data publishing thread is joined before destruction
        }
        if (ekf_data_file_.is_open()) {
            ekf_data_file_.close();
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

    // Method to publish H-SLAM data
    void publishHSLAMData(double timestamp, double x, double y, double z)
    {
        // Create a new Odometry message
        auto msg = std::make_unique<nav_msgs::msg::Odometry>();
        msg->header.stamp = this->now();  // Set the current time as the message timestamp
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
        msg->header.stamp = this->now();  // Set the current time as the message timestamp
        msg->header.frame_id = "gps";  // Set the frame id to "gps"
        msg->pose.pose.position.x = x_gps;
        msg->pose.pose.position.y = y_gps;
        msg->pose.pose.position.z = z_gps;

        // Publish the message
        gps_publisher_->publish(std::move(msg));

        // Log the published GPS data
        RCLCPP_INFO(this->get_logger(), "Published GPS data: %.0f %.16f %.16f %.15f", timestamp, x_gps, y_gps, z_gps);
    }

    // Callback method for EKF data
    void ekfCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Check if data publishing is done before writing to the file
        if (ekf_data_file_.is_open() && !data_publishing_done_) {
            ekf_data_file_ << "Timestamp: " << msg->header.stamp.sec << "." << msg->header.stamp.nanosec
                           << ", Position: [" << msg->pose.pose.position.x << ", " << msg->pose.pose.position.y << ", " << msg->pose.pose.position.z << "]" << std::endl;
        }
    }

    // Publishers for H-SLAM and GPS data
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr hslam_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr gps_publisher_;
    
    // Subscriber for EKF data
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ekf_subscriber_;

    // File stream for saving EKF data
    std::ofstream ekf_data_file_;

    // Thread for publishing data
    std::thread data_thread_;

    // Flag to indicate if data publishing is done
    std::atomic<bool> data_publishing_done_;
};

// Main function to initialize the ROS 2 system and run the node
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);  // Initialize the ROS 2 system

    rclcpp::spin(std::make_shared<MixedDataPublisher>());  // Create and run the MixedDataPublisher node

    rclcpp::shutdown();  // Shutdown the ROS 2 system
    return 0;  // Return success status
}
