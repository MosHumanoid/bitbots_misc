import rospy
import rostopic
import inflect
from python_qt_binding.QtWidgets import QComboBox
from python_qt_binding.QtCore import QTimer, pyqtSignal
from humanoid_league_msgs.msg import GameState


class NamespaceSelect(QComboBox):
    namespaceChanged = pyqtSignal(str)
    availableNamespacesChanged = pyqtSignal(list)

    def __init__(self, parent):
        super(NamespaceSelect, self).__init__(parent)
        self.parent = parent

        self.__update_timer = QTimer(self)
        self.__update_timer.timeout.connect(self.__update_namespaces)
        self.__update_timer.start(2000)

        self.__known_namespaces = []
        self.activated.connect(self.__on_item_selected)

    def __update_namespaces(self):
        topics = rostopic.find_by_type("humanoid_league_msgs/GameState")
        namespaces = []

        for t in topics:
            ns = "/".join(t.split("/")[:-1])
            if ns == "":
                ns = "/"
            namespaces.append(ns)

        if namespaces == self.__known_namespaces:
            return
        
        if "/" in namespaces and len(namespaces) > 1:
            rospy.logwarn_once("multiple namespaces found even though one of them is /. "
                               "Live tool might not work properly")

        self.__known_namespaces = namespaces
        self.availableNamespacesChanged.emit(namespaces)
        self.parent.namespaceSelect.clear()
        for ns in namespaces:
            self.parent.namespaceSelect.addItem(ns)

        rospy.loginfo("available namespaces updated", logger_name=self.__class__.__name__)
        self.__on_item_selected(0)

    def __on_item_selected(self, i):
        """:type item: int"""
        self.namespaceChanged.emit(self.__known_namespaces[i])
