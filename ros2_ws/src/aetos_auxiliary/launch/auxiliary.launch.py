from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
        ld: LaunchDescription = LaunchDescription()

        serial_com = Node (package="aetos_auxiliary",
                                    executable="serial_com",
                                    name="serial_com"
                                    )        
        arbitration = Node (package="aetos_auxiliary",
                                    executable="arbitration",
                                    name="arbitration"
                                    )
        
        ld.add_action(serial_com)
        ld.add_action(arbitration)

        return ld