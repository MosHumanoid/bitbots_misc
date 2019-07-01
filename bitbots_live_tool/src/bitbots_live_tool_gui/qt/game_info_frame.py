import rospy
import rostopic
import inflect
from python_qt_binding.QtWidgets import QFrame
from python_qt_binding.QtCore import QTimer, pyqtSlot
from humanoid_league_msgs.msg import GameState


class GameInfoFrame(QFrame):
    def __init__(self, parent, *args, **kwargs):
        super(GameInfoFrame, self).__init__(parent, *args, **kwargs)
        self.parent = parent

        self.namespace = None   # type: str
        self.subscriber = None  # type: rospy.Subscriber

    def __update_subscriber(self):
        topics = [t for t in rostopic.find_by_type("humanoid_league_msgs/GameState")
                  if str(t).startswith(self.namespace)]

        if len(topics) > 1:
            rospy.logwarn("{} topics are publishing humanoid_league_msgs/GameState msgs. "
                          "Choosing first ({}) to listen to".format(len(topics), topics[0]),
                          logger_name=self.__class__.__name__)

        if self.subscriber is None or self.subscriber.name != topics[0]:
            if self.subscriber is not None:
                self.subscriber.unregister()
            self.subscriber = rospy.Subscriber(topics[0], GameState, self.__gamestate_cb,
                                               queue_size=5, tcp_nodelay=True)

    def __gamestate_cb(self, msg):
        """:type msg: GameState"""
        self.setHalftime(1 if msg.firstHalf else 2)

        totalTime = 10 * 60
        self.setTime((totalTime - msg.secondsRemaining) / 60, msg.secondsRemaining % 60)

        self.setScore(msg.ownScore, msg.rivalScore)

    @pyqtSlot(int)
    def setHalftime(self, halftime):
        """
        Set current halftime number (1st, 2nd, ...)
        :type halftime: int
        """
        e = inflect.engine()
        self.parent.halftimeLabel.setText("{} halftime".format(e.ordinal(halftime)))

    @pyqtSlot(int, int)
    def setTime(self, minutes, seconds):
        """
        Set current gametime
        :type minutes: int
        :type seconds: int
        """
        self.parent.timeLabel.setText("{}:{} / 10:00".format(minutes, seconds))
        rospy.logdebug("set time to '{}:{} / 10:00'".format(minutes, seconds), logger_name=self.__class__.__name__)

    @pyqtSlot(int, int)
    def setScore(self, us, them):
        """
        Set current game score meaning how many goals each team scored
        :param us: How many goals our team scored
        :type us: int
        :param them: How many goals the enemy team scored
        :type us: int
        """
        self.parent.scoreLabel.setText("{}:{}".format(us, them))
        rospy.logdebug("set score to '{}:{}'".format(us, them), logger_name=self.__class__.__name__)

    @pyqtSlot(str)
    def setNamespace(self, ns):
        """
        Set namsepace under which a GameState message is published
        :type ns: str
        """
        self.namespace = ns
        self.__update_subscriber()
        rospy.logdebug("set topic namespace to {}".format(ns), logger_name=self.__class__.__name__)
