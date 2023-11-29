// Copyright 2023 Tarun Trilokesh
//
// Licensed under the MIT License (MIT); you may not use this file except in
// compliance with the License. You may obtain a copy of the License at
// https://opensource.org/licenses/MIT
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file walker.hpp
 * @author Tarun Trilokesh
 * @date 11/23/2023
 * @brief Header file for the Walker class
 *
 * Declares the Walker class for robot navigation based on sensor inputs from 
 * lidar and depth camera.
 */

#ifndef _HOME_TARUN_ROS2_HUMBLE_WS_SRC_ROOMBA_WALKER_INCLUDE_WALKER_HPP_
#define _HOME_TARUN_ROS2_HUMBLE_WS_SRC_ROOMBA_WALKER_INCLUDE_WALKER_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

enum State {
    MOVING_FORWARD,
    TURNING_LEFT,
    TURNING_RIGHT
};

/**
 * @class Walker
 * @brief Controls robot navigation using sensor inputs
 *
 * Subscribes to lidar and depth camera data topics and publishes
 * velocity commands for robot control.
 */
class Walker : public rclcpp::Node {
 public:
    Walker();

 private:
    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void controlRobot();

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::
                                        SharedPtr lidar_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;

    bool obstacle_detected_;
    State state_;
};

#endif  // _HOME_TARUN_ROS2_HUMBLE_WS_SRC_ROOMBA_WALKER_INCLUDE_WALKER_HPP_
