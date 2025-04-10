from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
        ld: LaunchDescription = LaunchDescription()

        kinematic_node = Node (package="aetos_kinematic",
                                    executable="velocity_conversion",
                                    name="velocity_conversion"
                                    )
        
        ld.add_action(kinematic_node)

        return ld