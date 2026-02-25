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

    zed_cam_launch_path = os.path.join(get_package_share_directory('drone_odometry'), 'launch', 'zed_camera.launch.py')

    parameters=[{'frame_id':'base_link',
                 'odom_frame_id':'odom',
                 'subscribe_rgbd':True,
                 'approx_sync':False,
                 'wait_imu_to_init':True,
                 'initial_pose' :'0 0 0 0 0 0'}]

                #  'pub_loc_pose_only_when_localizing':False 1Hz

    remappings=[('imu', '/zed_front/zed_node/imu/data')]

    # remappings.append(('odom', '/zed_front/zed_node/odom'))
    remappings.append(('odom', '/px4/odometry/out'))

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
            package='drone_odometry',
            executable='px4_tf_pub',
            name='px4_tf_pub',
            output='screen',
            parameters=[
                {
                    'px4_odom_frame_id': "odom",
                    'publish_tf': True,
                    'feed_twist_to_px4': True,
                    'odom_child_is_not_base_link': True,
                    'odom_parent_is_not_odom': True, 
                    'vio_desired_parent_frame_id': "odom" # if odom_parent_is_not_odom is set to true, this frame is uset to compute offset for parent of odom to feed into px4
                }
            ],
            remappings=[
                ('/odometry/filtered', '/zed_front/zed_node/odom')
            ],
            arguments=['--log-level', 'info']
        ),  
        
        ### RTABMAP ###

        

        # Sync rgb/depth/camera_info together
        launch_ros.actions.Node(   
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            parameters=parameters,
            remappings=[('rgb/image', '/zed_front/zed_node/rgb/image_rect_color'),
                        ('rgb/camera_info', '/zed_front/zed_node/rgb/camera_info'),
                        ('depth/image', '/zed_front/zed_node/depth/depth_registered')]),

        # Visual odometry
        # Node(
        #     package='rtabmap_odom', executable='rgbd_odometry', output='screen',
        #     condition=UnlessCondition(LaunchConfiguration('use_zed_odometry')),
        #     parameters=parameters,
        #     remappings=remappings,),

        # VSLAM
        launch_ros.actions.Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d']),


    ])