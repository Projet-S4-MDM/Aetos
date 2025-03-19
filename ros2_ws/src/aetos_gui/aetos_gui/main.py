from threading import Thread 
import sys
import signal

import PyQt5
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QShortcut
from PyQt5.QtGui import QKeySequence
from PyQt5 import uic
import rclpy
from rclpy.executors import MultiThreadedExecutor
from aetos_gui.ui_node import UINode

class MainWindow(QMainWindow):
    def __init__(self, ui_node, executor):
        super(MainWindow, self).__init__()
        self.ui_node = ui_node  # Référence au nœud ROS2
        self.executor = executor  # Exécuteur multi-thread pour ROS2

        # Ajout d'un raccourci clavier pour fermer l'application
        shortcut = QShortcut(QKeySequence("Ctrl+W"), self)
        shortcut.activated.connect(self.close_application)

        # Chargement de l'interface utilisateur depuis un fichier .ui
        resources_directory = self.ui_node.get_resources_directory('aetos_gui')
        uic.loadUi(resources_directory + "main_window.ui", self)

    def closeEvent(self, event):
        """Gère la fermeture de l'application."""
        self.close_application()

    def close_application(self):
        """Ferme l'application et arrête le nœud ROS2."""
        self.ui_node.destroy_node()
        self.executor.shutdown()
        QApplication.quit()


def main(args=None):
    """Point d'entrée principal de l'application."""
    rclpy.init(args=args)
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    # Activation du support des écrans 4K
    if hasattr(PyQt5.QtCore.Qt, 'AA_EnableHighDpiScaling'):
        PyQt5.QtWidgets.QApplication.setAttribute(PyQt5.QtCore.Qt.AA_EnableHighDpiScaling, True)
    if hasattr(PyQt5.QtCore.Qt, 'AA_UseHighDpiPixmaps'):
        PyQt5.QtWidgets.QApplication.setAttribute(PyQt5.QtCore.Qt.AA_UseHighDpiPixmaps, True)

    ui_node = UINode()
    executor = MultiThreadedExecutor()
    executor.add_node(ui_node)

    app = QApplication(sys.argv)
    window = MainWindow(ui_node, executor)

    # Démarrer l'exécution du nœud ROS2 dans un thread séparé
    thread = Thread(target=executor.spin)
    thread.start()

    window.show()
    sys.exit(app.exec_())

    
if __name__ == '__main__':
    main()
