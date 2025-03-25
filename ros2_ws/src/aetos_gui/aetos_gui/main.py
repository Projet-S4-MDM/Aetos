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
from aetos_msgs.msg import VelocityArbitration
from aetos_msgs.srv import VelocityArbitration as VelocityArbitrationSrv

class MainWindow(QMainWindow):
    def __init__(self, ui_node, executor):
        super(MainWindow, self).__init__()
        self.ui_node = ui_node  
        self.executor = executor  
        
        shortcut = QShortcut(QKeySequence("Ctrl+W"), self)
        shortcut.activated.connect(self.close_application)


        resources_directory = self.ui_node.get_resources_directory('aetos_gui')
        uic.loadUi(resources_directory + "main_window.ui", self)
        
        self.rb_manuel = self.findChild(PyQt5.QtWidgets.QRadioButton, "rb_manuel")
        self.rb_automatique = self.findChild(PyQt5.QtWidgets.QRadioButton, "rb_automatique")
        
        self.rb_manuel.toggled.connect(self.update_arbitration)
        self.rb_automatique.toggled.connect(self.update_arbitration)
        
    def update_arbitration(self):

        if self.rb_manuel.isChecked():
            mode = VelocityArbitration.TELEOP
        elif self.rb_automatique.isChecked():
            mode = VelocityArbitration.AUTONOMUS
        else:
            mode = VelocityArbitration.NONE

        client = self.ui_node.create_client(VelocityArbitrationSrv, "aetos/communication/set_arbitration")
        if not client.wait_for_service(timeout_sec=1.0):
            print("Service not available")
            return
    
        request = VelocityArbitrationSrv.Request()
        request.target_arbitration = VelocityArbitration()  
        request.target_arbitration.arbitration = mode       

        future = client.call_async(request)
        future.add_done_callback(self.arbitration_response)

    
    def arbitration_response(self, future):
        try:
            response = future.result()
            print(f"Arbitration set to {response.current_arbitration.arbitration}")
        except Exception as e:
            print(f"Service call failed: {e}")

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

   
    if hasattr(PyQt5.QtCore.Qt, 'AA_EnableHighDpiScaling'):
        PyQt5.QtWidgets.QApplication.setAttribute(PyQt5.QtCore.Qt.AA_EnableHighDpiScaling, True)
    if hasattr(PyQt5.QtCore.Qt, 'AA_UseHighDpiPixmaps'):
        PyQt5.QtWidgets.QApplication.setAttribute(PyQt5.QtCore.Qt.AA_UseHighDpiPixmaps, True)

    ui_node = UINode()
    executor = MultiThreadedExecutor()
    executor.add_node(ui_node)

    app = QApplication(sys.argv)
    window = MainWindow(ui_node, executor)

    
    thread = Thread(target=executor.spin)
    thread.start()

    window.show()
    sys.exit(app.exec_())

    
if __name__ == '__main__':
    main()