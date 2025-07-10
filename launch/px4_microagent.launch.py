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

    return LaunchDescription([

        
        # ExecuteProcess(
        #     cmd=[
        #         'MicroXRCEAgent',  # Nome eseguibile
        #         'serial',           # Modalità
        #         '--dev', '/dev/ttyTHS1',  # Device seriale
        #         '-b', '921600'       # Baud rate
        #     ],
        #     output='screen',  # Mostra l'output nel terminale
        #     shell=False       # Non usare la shell (sicurezza)
        # )

        ExecuteProcess(
            cmd=['MicroXRCEAgent', 'serial', '--dev', '/dev/ttyTHS1', '-b', '921600'],
            name='micro_agent_px4',  
            output='both',
        ),

    ])