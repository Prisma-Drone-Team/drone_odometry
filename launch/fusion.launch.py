import copy
from launch import LaunchDescription
import launch_ros.actions
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import sys
import pathlib
import os
import yaml
sys.path.append(str(pathlib.Path(__file__).parent.absolute()))

def generate_launch_description():

 
    return LaunchDescription([

        ## FUSION
        launch_ros.actions.Node(
            package = "tf2_ros",
            executable = "static_transform_publisher",
            name = "map_to_odom",
            arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "map", "odom"]
        ),
        
        launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(get_package_share_directory("drone_odometry2"), 'config', 'ukf_params.yaml')],
        ),
    ])