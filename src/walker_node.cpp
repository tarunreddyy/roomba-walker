// Copyright 2023 Tarun Trilokesh
//
// Licensed under the MIT License (MIT); you may not use this file except in
// compliance with the License. You may obtain a copy of the License at
//
// https://opensource.org/licenses/MIT
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file walker.cpp
 * @author Tarun Trilokesh
 * @date 11/23/2023
 * @brief Implementation file for the Walker class
 *
 * Implements the Walker class for robot navigation based on lidar and depth camera sensor inputs.
 */

#include "walker.hpp"

/**
 * @brief Constructor for Walker class
 * 
 * Initializes the node, lidar and depth camera subscribers, and velocity publisher.
 */
Walker::Walker() : Node("walker"), obstacle_detected_(false) {
    lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                        "/scan", 10, std::bind(&Walker::lidarCallback,
                        this, std::placeholders::_1));
    velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>
                            ("/cmd_vel", 10);
}

/**
 * @brief Callback function for lidar data
 * 
 * Processes incoming lidar data to detect obstacles and adjust robot's path.
 * @param msg The incoming lidar data message
 */
void Walker::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Define angular ranges for left, right, and front
    double left_angle_range = 60.0 * M_PI / 180.0;  // 60 degrees to the left
    double right_angle_range = -60.0 * M_PI / 180.0;  // 60 degrees to the right
    double front_angle_range = 30.0 * M_PI / 180.0;  // 30 degrees front

    // Initialize obstacle detection flags
    bool left_obstacle = false;
    bool right_obstacle = false;
    bool front_obstacle = false;

    // Process lidar data
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
        double angle = msg->angle_min + i * msg->angle_increment;
        double distance = msg->ranges[i];

        // Check for obstacles within a certain range
        if (distance < 0.5) {  // 0.5 meters
            if (angle >= left_angle_range) {
                left_obstacle = true;
            } else if (angle <= right_angle_range) {
                right_obstacle = true;
            } else if (std::abs(angle) <= front_angle_range) {
                front_obstacle = true;
            }
        }
    }

    // Update state based on obstacle detection
    if (front_obstacle) {
        state_ = (left_obstacle && !right_obstacle) ?
                    TURNING_RIGHT : TURNING_LEFT;
    } else {
        state_ = MOVING_FORWARD;
    }

    controlRobot();
}

/**
 * @brief Controls the robot's movement based on sensor data
 * 
 * Publishes velocity commands based on detected obstacles.
 */
void Walker::controlRobot() {
    auto twist = geometry_msgs::msg::Twist();

    switch (state_) {
        case MOVING_FORWARD:
            twist.linear.x = 0.2;  // Forward speed
            twist.angular.z = 0.0;  // No rotation
            break;
        case TURNING_LEFT:
            twist.linear.x = 0.0;  // Stop
            twist.angular.z = 0.5;  // Turn left
            break;
        case TURNING_RIGHT:
            twist.linear.x = 0.0;  // Stop
            twist.angular.z = -0.5;  // Turn right
            break;
    }
    velocity_publisher_->publish(twist);
}

int main(int argc, char **argv) {
    /**
     * @brief Main function to initialize and run the Walker node
     * 
     * Initializes the ROS2 system, creates a Walker object, and keeps
     * spinning until ROS2 shutdown is requested.
     */
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Walker>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
