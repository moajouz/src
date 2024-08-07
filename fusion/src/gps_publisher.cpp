#include "rclcpp/rclcpp.hpp"  // Include the main header file for ROS 2 C++ API
#include "sensor_msgs/msg/nav_sat_fix.hpp"  // Include the header for GPS data message type
#include <fstream>  // Include the header for file stream operations
#include <sstream>  // Include the header for string stream operations

using namespace std::chrono_literals;  // Use chrono literals for time durations

class GPSPublisher : public rclcpp::Node  // Define a class that inherits from rclcpp::Node
{
public:
    GPSPublisher() : Node("gps_publisher")  // Constructor initializes the node with the name "gps_publisher"
    {
        // Create a publisher for the "gps_data" topic with a queue size of 10
        publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("gps_data", 10);

        // Define the path to the CSV file containing the GPS data
        std::string csv_path = "/home/mooo/aub/datasets/modified/ficosa5_odometry.csv";
        loadGPSData(csv_path);  // Load the GPS data from the CSV file

        // Create a timer that triggers the publishGPSData method every 300 milliseconds
        timer_ = this->create_wall_timer(300ms, std::bind(&GPSPublisher::publishGPSData, this));
    }

private:
    // Method to load GPS data from a CSV file
    void loadGPSData(const std::string& csv_path)
    {
        std::ifstream file(csv_path);  // Open the CSV file
        if (!file.is_open()) {  // Check if the file was opened successfully
            RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file: %s", csv_path.c_str());
            return;
        }

        std::string line;
        std::getline(file, line); // Skip the header line

        // Read the file line by line
        while (std::getline(file, line)) {
            std::istringstream iss(line);  // Create a string stream for the current line
            double latitude, longitude, altitude, timestamp;
            char comma;

            // Skip columns 0-3 (assuming 0-indexed)
            for (int i = 0; i < 4; ++i) {
                iss.ignore(std::numeric_limits<std::streamsize>::max(), ',');
            }
            
            // Read columns 4, 5, 6 (latitude, longitude, altitude)
            iss >> latitude >> comma >> longitude >> comma >> altitude;

            // Skip to the timestamp column (fourth column after altitude)
            for (int i = 0; i < 4; ++i) {
                iss.ignore(std::numeric_limits<std::streamsize>::max(), ',');
            }

            // Read the timestamp
            iss >> timestamp;

            // Store GPS data for the current row in a struct
            GPSData gps_data;
            gps_data.latitude = latitude;
            gps_data.longitude = longitude;
            gps_data.altitude = altitude;
            gps_data.timestamp = timestamp;

            // Add the current row's data to the list
            gps_data_list_.push_back(gps_data);
        }

        file.close();  // Close the CSV file
    }

    // Method to publish GPS data
    void publishGPSData()
    {
        static size_t idx = 0;  // Static variable to keep track of the current index

        if (idx >= gps_data_list_.size()) {  // Check if all data has been published
            RCLCPP_WARN(this->get_logger(), "No more GPS data to publish.");
            return;
        }

        // Get GPS data from the list at the current index
        GPSData& gps_data = gps_data_list_[idx];

        // Create a new NavSatFix message
        auto msg = std::make_unique<sensor_msgs::msg::NavSatFix>();
        msg->latitude = gps_data.latitude;
        msg->longitude = gps_data.longitude;
        msg->altitude = gps_data.altitude;

        // Publish the message
        publisher_->publish(std::move(msg));

        // Log the published GPS data
        RCLCPP_INFO(this->get_logger(), "Published GPS data: Latitude=%.8f, Longitude=%.8f, Altitude=%.3f, Timestamp=%.0f",
                    gps_data.latitude, gps_data.longitude, gps_data.altitude, gps_data.timestamp);

        // Increment the index for the next data publication
        ++idx;
    }

private:
    // Structure to store GPS data
    struct GPSData {
        double latitude;
        double longitude;
        double altitude;
        double timestamp;
    };

    // List to store the GPS data read from the CSV file
    std::vector<GPSData> gps_data_list_;

    // Publisher for the GPS data
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_;

    // Timer to trigger the publishGPSData method
    rclcpp::TimerBase::SharedPtr timer_;
};

// Main function to initialize the ROS 2 system and run the node
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);  // Initialize the ROS 2 system

    rclcpp::spin(std::make_shared<GPSPublisher>());  // Create and run the GPSPublisher node

    rclcpp::shutdown();  // Shutdown the ROS 2 system
    return 0;  // Return success status
}
