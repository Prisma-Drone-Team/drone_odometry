import copy
from launch import LaunchDescription
import launch_ros.actions
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import sys
import pathlib
import os
import yaml
sys.path.append(str(pathlib.Path(__file__).parent.absolute()))

def generate_launch_description():

    # pkg_path = get_package_share_directory( 'gz_drone_bringup' )
    # yaml_file = os.path.join( pkg_path, 'config', 'classic_cfg.yaml' )
    # with open(yaml_file, 'r') as file:
    #     config = yaml.load(file, Loader=yaml.FullLoader)
    #     params = config['odom_republisher_simu']['ros__parameters'] 

    zed_cam_launch_path = os.path.join(get_package_share_directory('drone_odometry2'), 'launch', 'zed_camera.launch.py')


    return LaunchDescription([

        ## FRONT ZED
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(zed_cam_launch_path),
            launch_arguments={  'camera_model': 'zedm',
                                'publish_map_tf': 'false',
                                'publish_tf': 'false',
                                'odometry_frame': 'zed_front_odom',
                                'serial_number': '18758846',
                                'camera_name': 'zed_front' }.items()
        ),
        launch_ros.actions.Node(
            package = "tf2_ros",
            executable = "static_transform_publisher",
            name = "base_link_to_zed_front_camera_link",
            arguments=["0.2", "0.0", "-0.07", "0", "0", "0", "base_link", "zed_front_camera_link"]
        ),
        launch_ros.actions.Node(
            package = "tf2_ros",
            executable = "static_transform_publisher",
            name = "odom_to_zed_front_odom",
            arguments=["0.2", "0.0", "-0.07", "0", "0", "0", "odom", "zed_front_odom"]
        ),


        #launch microros agent
        ExecuteProcess(
            cmd=['MicroXRCEAgent', 'serial', '--dev', '/dev/ttyTHS1', '-b', '921600'],
            name='micro_agent_px4',  
            output='both',
        ),

        launch_ros.actions.Node(
            package='drone_odometry2',
            executable='px4_tf_pub',
            name='px4_tf_pub',
            output='screen',
            parameters=[
                {
                    'px4_odom_frame_id': "zed_front_odom",
                    'publish_tf': True,
                    'feed_twist_to_px4': True,
                    'odom_parent_is_not_map': False, #new idea
                    'odom_child_is_not_base_link': True,
                }
            ],
            remappings=[
                ('/odometry/filtered', '/zed_front/zed_node/odom')
            ],
            arguments=['--log-level', 'info']
        )

        
        

    ])