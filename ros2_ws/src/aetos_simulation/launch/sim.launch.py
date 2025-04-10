from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
        ld: LaunchDescription = LaunchDescription()

        sim_node = Node (package="aetos_simulation",
                                    executable="sim",
                                    name="sim"
                                    )
        
        ld.add_action(sim_node)

        return ld
