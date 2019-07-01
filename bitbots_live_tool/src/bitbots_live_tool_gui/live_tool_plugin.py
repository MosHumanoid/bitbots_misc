import os

import rospkg
import rospy
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi


class LiveToolPlugin(Plugin):

    def __init__(self, context):
        super(LiveToolPlugin, self).__init__(context)

        rospack = rospkg.RosPack()
        resource_dir = os.path.join(rospack.get_path("bitbots_live_tool"), "resources")
        self.root_widget = loadUi(os.path.join(resource_dir, "main.ui"))

        # connect different handlers to signals
        self.root_widget.namespaceSelect.namespaceChanged.connect(self.enable_all)

        context.add_widget(self.root_widget)

    def enable_all(self, *args):
        self.root_widget.tabWidget.setEnabled(True)
        self.root_widget.gameInfoFrame.setEnabled(True)
