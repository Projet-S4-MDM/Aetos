import sys
import signal
from threading import Thread

import PyQt5
from PyQt5.QtWidgets import QApplication, QMainWindow, QShortcut, QPushButton, QRadioButton, QLabel
from PyQt5.QtGui import QKeySequence, QPixmap, QImage
from PyQt5.QtCore import pyqtSignal, QObject, Qt
from PyQt5 import uic

import rclpy
from rclpy.executors import MultiThreadedExecutor
from aetos_gui.ui_node import UINode
from aetos_msgs.msg import Velocity, EffectorPosition
from aetos_msgs.msg import VelocityArbitration, EncoderArbitration
from aetos_msgs.srv import VelocityArbitration as VelocityArbitrationSrv
from aetos_msgs.srv import EncoderArbitration as EncoderArbitrationSrv
from sensor_msgs.msg import Image
from aetos_msgs.msg import Velocity

from cv_bridge import CvBridge
import cv2

class DataSubscriberNode(UINode, QObject):
    video_signal = pyqtSignal(QPixmap)

    def __init__(self):
        UINode.__init__(self, "data_subscriber")
        QObject.__init__(self)

        
        self.velocity_subscription = self.create_subscription(
            Velocity,
            "aetos/cam/velocity",
            self.velocity_callback,
            10
        )

        
        self.video_subscription = self.create_subscription(
            Image,
            "aetos/cam/video",
            self.video_callback,
            10
        )

        self.bridge = CvBridge()

    def velocity_callback(self, msg: Velocity):
        """ Affiche la vitesse reçue dans le terminal """
        

    def video_callback(self, msg: Image):
        """ Convertit et émet le flux vidéo """
        try:
            
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            
            height, width, _ = cv_image.shape
            bytes_per_line = 3 * width
            q_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_BGR888)
            q_pixmap = QPixmap.fromImage(q_image)

            
            self.video_signal.emit(q_pixmap)

        except Exception as e:
            self.get_logger().error(f"Erreur lors de la conversion de l'image: {e}")



class MainWindow(QMainWindow):
    def __init__(self, ui_node, executor):
        super(MainWindow, self).__init__()
        self.ui_node = ui_node
        self.executor = executor

        
        shortcut = QShortcut(QKeySequence("Ctrl+W"), self)
        shortcut.activated.connect(self.close_application)

        
        resources_directory = self.ui_node.get_resources_directory('aetos_gui')
        uic.loadUi(resources_directory + "main_window.ui", self)

        
        self.rb_manuel = self.findChild(QRadioButton, "rb_manuel")
        self.rb_automatique = self.findChild(QRadioButton, "rb_automatique")
        self.rb_simulation = self.findChild(QRadioButton, "rb_simulation")
        self.rb_real = self.findChild(QRadioButton, "rb_real")
        self.pb_arret = self.findChild(QPushButton, "pb_arret")

        
        self.video_label = QLabel(self)
        self.video_label.setGeometry(250, 70, 640, 480)  
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setStyleSheet("border: 2px solid black;")

        
        self.rb_manuel.toggled.connect(self.update_velocity_arbitration)
        self.rb_automatique.toggled.connect(self.update_velocity_arbitration)
        self.rb_simulation.toggled.connect(self.update_encoder_arbitration)
        self.rb_real.toggled.connect(self.update_encoder_arbitration)
        self.pb_arret.clicked.connect(self.close_application)
        
        self.lbl_vx = self.findChild(QLabel, "Vx")
        self.lbl_vy = self.findChild(QLabel, "Vy")
        self.lbl_vz = self.findChild(QLabel, "Vz")
        
        self.lbl_px = self.findChild(QLabel, "position_x")
        self.lbl_py = self.findChild(QLabel, "position_y")
        self.lbl_pz = self.findChild(QLabel, "position_z")
        
        self.ui_node.create_subscription(Velocity, 'aetos/control/velocity', self.velocity_callback, 10)
        self.subscription = self.ui_node.create_subscription(EffectorPosition, 'aetos/control/position', self.effector_position_callback, 10)

        self.ui_node.video_signal.connect(self.update_video)

    def update_video(self, pixmap: QPixmap):
        """ Met à jour l'affichage vidéo """
        self.video_label.setPixmap(pixmap)
        
    def velocity_callback(self, msg):
        self.lbl_vx.setText(f"{msg.velocity_x:.2f}")
        self.lbl_vy.setText(f"{msg.velocity_y:.2f}")
        self.lbl_vz.setText(f"{msg.velocity_z:.2f}")
    
    def effector_position_callback(self, msg):
        self.lbl_px.setText(f"{msg.position_x:.2f}")
        self.lbl_py.setText(f"{msg.position_y:.2f}")
        self.lbl_pz.setText(f"{msg.position_z:.2f}")

    def update_velocity_arbitration(self):
        """ Gestion de l'arbitrage de vitesse """
        if self.rb_manuel.isChecked():
            mode = VelocityArbitration.TELEOP
        elif self.rb_automatique.isChecked():
            mode = VelocityArbitration.AUTONOMUS  
        else:
            mode = VelocityArbitration.NONE

        self.send_velocity_arbitration_request(mode)

    def update_encoder_arbitration(self):
        """ Gestion de l'arbitrage encodeur """
        if self.rb_simulation.isChecked():
            mode = EncoderArbitration.SIM_ENCODER
        elif self.rb_real.isChecked():
            mode = EncoderArbitration.MOTOR_ENCODER
        else:
            mode = EncoderArbitration.NONE

        self.send_encoder_arbitration_request(mode)

    def send_velocity_arbitration_request(self, mode):
        """ Envoi de la demande d'arbitrage de vitesse """
        client = self.ui_node.create_client(VelocityArbitrationSrv, "aetos/communication/set_velocityArbitration")
        if not client.wait_for_service(timeout_sec=1.0):
            print("Velocity arbitration service not available")
            return

        request = VelocityArbitrationSrv.Request()
        request.target_arbitration = VelocityArbitration()
        request.target_arbitration.arbitration = mode

        future = client.call_async(request)
        future.add_done_callback(self.arbitration_response)

    def send_encoder_arbitration_request(self, mode):
        """ Envoi de la demande d'arbitrage encodeur """
        client = self.ui_node.create_client(EncoderArbitrationSrv, "aetos/communication/set_encoderArbitration")
        if not client.wait_for_service(timeout_sec=1.0):
            print("Encoder arbitration service not available")
            return

        request = EncoderArbitrationSrv.Request()
        request.target_arbitration = EncoderArbitration()
        request.target_arbitration.arbitration = mode

        future = client.call_async(request)
        future.add_done_callback(self.arbitration_response)

    def arbitration_response(self, future):
        """ Réponse des services """
        try:
            response = future.result()
            print(f"Arbitration set to {response.current_arbitration.arbitration}")
        except Exception as e:
            print(f"Service call failed: {e}")

    def close_application(self):
        """ Fermeture propre """
        print("Fermeture de l'application...")
        self.ui_node.destroy_node()
        self.executor.shutdown()
        QApplication.quit()
        sys.exit(0)



def main():
    rclpy.init()
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    
    node = DataSubscriberNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    
    ros_thread = Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    
    app = QApplication(sys.argv)
    window = MainWindow(node, executor)

    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
