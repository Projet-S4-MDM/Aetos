from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
        ld: LaunchDescription = LaunchDescription()

        cam_node = Node (package="aetos_cam",
                                    executable="camera",
                                    name="camera"
                                    )
        
        ld.add_action(cam_node)

        return ld