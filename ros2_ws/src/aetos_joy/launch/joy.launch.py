from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
        ld: LaunchDescription = LaunchDescription()

        joy_node = Node (package="aetos_joy",
                                    executable="joy_demux",
                                    name="joy_demux"
                                    )
        
        ld.add_action(joy_node)

        return ld
