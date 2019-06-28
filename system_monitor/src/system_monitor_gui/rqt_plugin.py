from qt_gui.plugin import Plugin
from python_qt_binding import loadUi


class SystemMonitorPlugin(Plugin):

    def __init__(self, context):
        super(SystemMonitorPlugin, self).__init__(context)
