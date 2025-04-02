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


# ============================
# Node ROS2 avec PyQt5 Signals
# ============================
class DataSubscriberNode(UINode, QObject):
    video_signal = pyqtSignal(QPixmap)

    def __init__(self):
        UINode.__init__(self, "data_subscriber")
        QObject.__init__(self)

        # Souscripteur pour les vecteurs de vitesse
        self.velocity_subscription = self.create_subscription(
            Velocity,
            "aetos/cam/velocity",
            self.velocity_callback,
            10
        )

        # Souscripteur pour le flux vidéo
        self.video_subscription = self.create_subscription(
            Image,
            "aetos/cam/video",
            self.video_callback,
            10
        )

        self.bridge = CvBridge()

    def velocity_callback(self, msg: Velocity):
        """ Affiche la vitesse reçue dans le terminal """
        #self.get_logger().info(f"Vitesse reçue: vx={msg.velocity_x}, vy={msg.velocity_y}, vz={msg.velocity_z}")

    def video_callback(self, msg: Image):
        """ Convertit et émet le flux vidéo """
        try:
            # Convertir l'image ROS2 en OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Conversion en QPixmap pour l'interface Qt
            height, width, _ = cv_image.shape
            bytes_per_line = 3 * width
            q_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_BGR888)
            q_pixmap = QPixmap.fromImage(q_image)

            # Émission du signal PyQt5 pour l'affichage
            self.video_signal.emit(q_pixmap)

        except Exception as e:
            self.get_logger().error(f"Erreur lors de la conversion de l'image: {e}")


# ============================
# Fenêtre principale Qt5
# ============================
class MainWindow(QMainWindow):
    def __init__(self, ui_node, executor):
        super(MainWindow, self).__init__()
        self.ui_node = ui_node
        self.executor = executor

        # Raccourci pour quitter avec Ctrl+W
        shortcut = QShortcut(QKeySequence("Ctrl+W"), self)
        shortcut.activated.connect(self.close_application)

        # Chargement de l'interface Qt existante
        resources_directory = self.ui_node.get_resources_directory('aetos_gui')
        uic.loadUi(resources_directory + "main_window.ui", self)

        # Connexion des éléments d'interface
        self.rb_manuel = self.findChild(QRadioButton, "rb_manuel")
        self.rb_automatique = self.findChild(QRadioButton, "rb_automatique")
        self.rb_simulation = self.findChild(QRadioButton, "rb_simulation")
        self.rb_real = self.findChild(QRadioButton, "rb_real")
        self.pb_arret = self.findChild(QPushButton, "pb_arret")

        # ✅ Zone pour afficher le flux vidéo (QLabel)
        self.video_label = QLabel(self)
        self.video_label.setGeometry(250, 70, 640, 480)  # Position + taille
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setStyleSheet("border: 2px solid black;")

        # Connexion des boutons aux méthodes
        self.rb_manuel.toggled.connect(self.update_velocity_arbitration)
        self.rb_automatique.toggled.connect(self.update_velocity_arbitration)
        self.rb_simulation.toggled.connect(self.update_encoder_arbitration)
        self.rb_real.toggled.connect(self.update_encoder_arbitration)
        self.pb_arret.clicked.connect(self.close_application)

        # Connexion du signal PyQt5
        self.ui_node.video_signal.connect(self.update_video)

    def update_video(self, pixmap: QPixmap):
        """ Met à jour l'affichage vidéo """
        self.video_label.setPixmap(pixmap)

    def update_velocity_arbitration(self):
        """ Gestion de l'arbitrage de vitesse """
        if self.rb_manuel.isChecked():
            mode = VelocityArbitration.TELEOP
        elif self.rb_automatique.isChecked():
            mode = VelocityArbitration.AUTONOMOUS  
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


# ============================
# Fonction principale
# ============================
def main():
    rclpy.init()
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    # Initialisation du node et de l'exécuteur
    node = DataSubscriberNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # Démarrage du thread ROS2
    ros_thread = Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    # Lancement de l'interface Qt
    app = QApplication(sys.argv)
    window = MainWindow(node, executor)

    window.show()
    sys.exit(app.exec_())


# ============================
# Lancement principal
# ============================
if __name__ == '__main__':
    main()
