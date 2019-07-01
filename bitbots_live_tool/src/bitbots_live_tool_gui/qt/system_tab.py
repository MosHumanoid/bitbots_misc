from typing import *
import rospy
import rostopic
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtCore import QTimer, pyqtSlot, Signal
from bitbots_msgs.msg import Workload
import pyqtgraph


class CpuWidget(QWidget):
    def __init__(self, parent):
        super(CpuWidget, self).__init__(parent)
        self.parent = parent

        self.namespace = None   # type: str
        self.subscriber = None  # type: rospy.Subscriber
        self.data = []     # type: List[Tuple[rospy.Time, float]]

        self.__render_timer = QTimer(self)
        self.__render_timer.timeout.connect(self.__render)
        self.__render_timer.start(500)
        self.__init_time = rospy.Time.now()

    @pyqtSlot(str)
    def setNamespace(self, ns):
        """
        Set namsepace under which a GameState message is published
        :type ns: str
        """
        self.namespace = ns
        rospy.logdebug("set topic namespace to {}".format(ns), logger_name=self.__class__.__name__)
        self.__update_subscriber()

    def __update_subscriber(self):
        """
        Update the internal subscriber to new namespace
        """
        topics = [t for t in rostopic.find_by_type("bitbots_msgs/Workload")
                  if str(t).startswith(self.namespace)]

        if len(topics) > 1:
            rospy.logwarn("{} topics are publishing bitbots_msgs/Workload msgs. "
                          "Choosing first ({}) to listen to".format(len(topics), topics[0]),
                          logger_name=self.__class__.__name__)
        elif len(topics) < 1:
            rospy.logerr("No topics publishing bitbots_msgs/Workload are known. "
                         "System information will not be displayed")
            return

        if self.subscriber is None or self.subscriber.name != topics[0]:
            if self.subscriber is not None:
                self.subscriber.unregister()
            self.subscriber = rospy.Subscriber(topics[0], Workload, self.__workload_cb,
                                               queue_size=5, tcp_nodelay=True)

        rospy.logdebug("updated subscriber to listen to {}".format(topics[0]), logger_name=self.__class__.__name__)

    def __workload_cb(self, workload):
        """:type workload: Workload"""
        self.data.append((rospy.Time.now(), float(workload.cpus[0].cpu_usage)))
        self.__shorten_data()

    def __shorten_data(self):
        if len(self.data) > 100 and self.data[-1][0] - self.data[0][0] > rospy.Duration(5):
            time_limit = self.data[-1][0] - rospy.Duration(5)
            self.data = [(time, val) for (time, val) in self.data
                         if time > time_limit]

    def __render(self):
        plot = self.findChild(pyqtgraph.PlotWidget, "cpuGraphicsView")

        x = []
        y = []
        for (time, val) in self.data:
            x.append((time - self.__init_time).to_sec())
            y.append(val)

        plot.plot(x, y)
