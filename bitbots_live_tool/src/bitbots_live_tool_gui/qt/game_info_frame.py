import rospy
import rostopic
import inflect
from python_qt_binding.QtWidgets import QFrame
from python_qt_binding.QtCore import QTimer
from humanoid_league_msgs.msg import GameState


class GameInfoFrame(QFrame):
    def __init__(self, parent):
        super(GameInfoFrame, self).__init__(parent)
        self.parent = parent

        self.subscriber = None  # type: rospy.Subscriber
        self.__update_timer = QTimer(self)
        self.__update_timer.timeout.connect(self.__update_subscribers)
        self.__update_timer.start(5000)

    def __update_subscribers(self):
        topics = rostopic.find_by_type("humanoid_league_msgs/GameState")

        if len(topics) > 1:
            rospy.logwarn("{} topics are publishing humanoid_league_msgs/GameState msgs. "
                          "Choosing first ({}) to listen to".format(len(topics), topics[0]))

        if self.subscriber is None or \
                self.subscriber.name != topics[0]:
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

    def setHalftime(self, halftime):
        """
        Set current halftime number (1st, 2nd, ...)
        :type halftime: int
        """
        e = inflect.engine()
        self.parent.halftimeLabel.setText("{} halftime".format(e.ordinal(halftime)))

    def setTime(self, minutes, seconds):
        """
        Set current gametime
        :type minutes: int
        :type seconds: int
        """
        self.parent.timeLabel.setText("{}:{} / 10:00".format(minutes, seconds))

    def setScore(self, us, them):
        """
        Set current game score meaning how many goals each team scored
        :param us: How many goals our team scored
        :type us: int
        :param them: How many goals the enemy team scored
        :type us: int
        """
        self.parent.scoreLabel.setText("{}:{}".format(us, them))
