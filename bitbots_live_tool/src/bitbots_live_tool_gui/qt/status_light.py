import rospy
import rostopic
import inflect
from python_qt_binding.QtWidgets import QWidget, QSizePolicy
from python_qt_binding.QtCore import QTimer
from humanoid_league_msgs.msg import GameState


class StatusLight(QWidget):

    def __init__(self, parent):
        super(StatusLight, self).__init__(parent)
        self.parent = parent

        sizePolicy = QSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred)
        sizePolicy.setWidthForHeight(True)
        self.setSizePolicy(sizePolicy)
