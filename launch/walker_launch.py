# Copyright 2023 Tarun Trilokesh
#
# Licensed under the MIT License (MIT); you may not use this file except in
# compliance with the License. You may obtain a copy of the License at
# https://opensource.org/licenses/MIT
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
File: walker_launch.py
Author: Tarun Trilokesh
Date: 11/23/2023
Description: Launch file for starting the Walker node, the Gazebo simulation, and optionally timed rosbag recording.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument, Shutdown, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare the launch argument for enabling rosbag recording
    record_rosbag_arg = DeclareLaunchArgument(
        'record_rosbag',
        default_value='false',
        description='Flag to enable rosbag recording'
    )
    
    # Retrieve the argument value
    record_rosbag = LaunchConfiguration('record_rosbag')

    # Path to the TurtleBot3 Gazebo launch file
    turtlebot3_gazebo_launch_file = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'launch',
        'turtlebot3_world.launch.py'
    )

    # Define the path to save rosbag files
    rosbag_save_path = os.path.join(
        'src', 'roomba_walker', 'results', 'rosbag'
    )

    # Rosbag record command with updated path
    rosbag_record_cmd = ExecuteProcess(
        condition=IfCondition(record_rosbag),
        cmd=['ros2', 'bag', 'record', '-a', '-o', rosbag_save_path, '--exclude', '/camera/*'],
        output='screen'
    )

    # Stop recording after 15 seconds
    stop_rosbag_cmd = TimerAction(
        period=30.0,
        actions=[Shutdown()],
        condition=IfCondition(record_rosbag)
    )

    return LaunchDescription([
        record_rosbag_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(turtlebot3_gazebo_launch_file),
            launch_arguments={'model': 'waffle_pi'}.items(),
        ),
        Node(
            package='roomba_walker',
            executable='walker_node',
            name='walker',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        rosbag_record_cmd,
        stop_rosbag_cmd
    ])
