import os
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

class UINode(Node):
    def __init__(self):
        super().__init__('ui_node')

    def get_resources_directory(self, package_name):
        """Retourne le chemin du dossier 'resource' où se trouvent les fichiers UI."""
        package_share_directory = get_package_share_directory(package_name)
        resource_directory = os.path.join(package_share_directory, "resource")

        if not os.path.exists(resource_directory):
            self.get_logger().warn(f"Le répertoire des ressources n'existe pas : {resource_directory}")

        return resource_directory + "/"
