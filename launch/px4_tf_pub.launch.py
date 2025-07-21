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

        launch_ros.actions.Node(
            package='drone_odometry2',
            executable='px4_tf_pub',
            name='px4_tf_pub',
            output='screen',
            parameters=[
                {
                    'px4_odom_frame_id': "odom", #frame of published tf and odom msg
                    'publish_tf': True,
                    'feed_twist_to_px4': True,
                    'odom_parent_is_not_odom': True, #if vio odom msg parent is not odom, use tf to add offset
                    'odom_child_is_not_base_link': True, #if vio odom msg child is not base_link, use tf to add offset
                }
            ],
            remappings=[
                ('/odometry/filtered', '/zed_front/zed_node/odom') #vio pc odometry to feed into px4
            ],
            arguments=['--log-level', 'info']
        )  

    ])