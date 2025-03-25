from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    
    return LaunchDescription([

        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('aetos_joy'), 'launch', 'joy.launch.py'])])),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('aetos_kinematic'), 'launch', 'kinematics.launch.py'])])),
    
        # IncludeLaunchDescription(
            # PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('aetos_simulation'), 'launch', 'sim.launch.py'])])),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('aetos_auxiliary'), 'launch', 'auxiliary.launch.py'])]))

    ])
    
# TODO
# 1. Fix sim shutting down on interrupt
# 2. Cerate full sim vs visualize sim for encoder topic management
# 4. Implement deadman