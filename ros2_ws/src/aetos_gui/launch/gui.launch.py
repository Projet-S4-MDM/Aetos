from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
        ld: LaunchDescription = LaunchDescription()

        gui_node = Node (package="aetos_gui",
                                    executable="main",
                                    name="main"
                                    )
        
        ld.add_action(gui_node)

        return ld