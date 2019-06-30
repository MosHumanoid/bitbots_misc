import os

import rospkg
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi


class LiveToolPlugin(Plugin):

    def __init__(self, context):
        super(LiveToolPlugin, self).__init__(context)

        rospack = rospkg.RosPack()
        resource_dir = os.path.join(rospack.get_path("bitbots_live_tool"), "resources")
        self.root_widget = loadUi(os.path.join(resource_dir, "main.ui"))

        context.add_widget(self.root_widget)