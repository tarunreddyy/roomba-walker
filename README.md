# Roomba Walker ROS2 Package

## Overview
The Roomba Walker package is a ROS2-based robot navigation system that mimics the behavior of a Roomba vacuum cleaner. It utilizes sensor data from lidar to navigate and avoid obstacles. This package is developed and tested on ROS2 Humble.

## Dependencies
- ROS2 Humble
- Gazebo (compatible with ROS2 Humble)
- TurtleBot3 packages and models

## Building the Package
1. Navigate to your ROS2 workspace (e.g., `ros2_ws`).
2. Clone the package into the `src` directory of your workspace.
3. Build the package using the following command:
   ```bash
   colcon build --packages-select roomba_walker --symlink-install
   ```

## Running the Simulation
1. Source the ROS2 environment:
   ```bash
   source /opt/ros/humble/setup.bash
   ```
2. Source your workspace:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```
3. Launch the simulation:
   ```bash
   ros2 launch roomba_walker walker_launch.py
   ```

## Recording Rosbag Files
- To record rosbag files during the simulation, run:
  ```bash
  ros2 launch roomba_walker walker_launch.py record_rosbag:=true
  ```
  This will record all topics except `/camera/*` topics for 30 seconds.

- To disable rosbag recording, simply run the simulation without the `record_rosbag` argument.

The rosbag files are saved in the `results` directory.

## Inspecting the Rosbag File
- To inspect the contents of a rosbag file navigate to the rosbag directory in results:
  ```bash
  ros2 bag info rosbag_0.db3
  ```

## Playing Back the Rosbag File
- To see the playback of the trutlebot navigation ensure you run the below command first in a different terminal:
  ```bash
  ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py 
  ```
- Navigate to the rosbag directory in results and play the rosbag file using:
  ```bash
  ros2 bag play rosbag_0.db3
  ```

## Linting the Code
This package follows the Google C++ Style Guide. The linting is done using `cpplint` and `cppcheck`.

- To run `cpplint`:
  ```bash
  cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order,-build/include_subdir $(find . -name \*.hpp -or -name \*.cpp) > results/cpplint_results.txt 2>&1
  ```
- To run `cppcheck`:
  ```bash
  cppcheck --enable=all --std=c++17 --suppress=missingIncludeSystem $(find . -name \*.hpp -or -name \*.cpp) 2> results/cppcheck_results.txt
  ```

The results of these checks are saved in the `results` directory.

## License
This project is licensed under the MIT License - see the LICENSE file for details.
