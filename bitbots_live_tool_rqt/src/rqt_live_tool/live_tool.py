import rospkg
import rospy
import time
import socket
import multiprocessing as mp
import yaml
import sys
import os

from python_qt_binding import loadUi
from python_qt_binding.QtCore import *

from python_qt_binding.QtWidgets import QMainWindow, QWidget, QTreeWidget, QTreeWidgetItem, QListWidgetItem, \
    QSlider, QGroupBox, QVBoxLayout, QLabel, QLineEdit, QListWidget, QAbstractItemView, QFileDialog, QDoubleSpinBox, \
    QMessageBox, QInputDialog, QShortcut

from rqt_gui_py.plugin import Plugin

import os

from defaultview import DefaultView
from information_tab import InformationTab
from single_field import SingleField
from quarter_field import QuarterField
from main_window import MainWindow
from label_pool import Arrowlabel

from status_msg import StatusMsg
from trajectory_msg import TrajectoryMsg
from detection_msg import DetectionMsg
from position_msg import PositionMsg

from name import Name


# A QT-Worker thread for reveiving udp messages and sending a signal so the UI can be updated
# The signal transmits the data to the UDP-parser finally updating the UI
class UdpWorker(QThread):
    signal = pyqtSignal(str)
    def __init__(self, socket, parent=None):
        super(UdpWorker, self).__init__(parent)
        self.sock = socket

    def run(self):
        while not rospy.is_shutdown():
            try:
                data, addr = self.sock.recvfrom(4096)
                self.signal.emit(data)
            except socket.timeout as t:
                print("Nothing received")
                None

    def handle_connect(self):
        request_handler(self)

    def handle_close(self):
        self.close()



class LiveTool(Plugin):
    RECEIVE_TOKEN = "IP_RECEIVE_FROM"
    PORT_TOKEN = "UDP_PORT"
    TIMEOUT_TOKEN = "RECEIVE_TIMEOUT"

    UDP_IP = "" # binds socket to this IP
    UDP_PORT = 5005
    UDP_TIMEOUT = 1.0

    UI_FILE = "livetool.ui"

    UDP_POSITION_MSG = PositionMsg.title
    UDP_STATUS_MSG = StatusMsg.title
    UDP_DETECTION_MSG = DetectionMsg.title
    UDP_TRAJECTORY_MSG = TrajectoryMsg.title

    UDP_ID_SPLIT = "::" # Messages arrive in this format: ROBOT_ID::MESSAGE_ID::DATA

    def __init__(self, context):
        super(LiveTool, self).__init__(context)

        # Set configuration from "ip_config.yaml"
        self.receiveIPconfig()

        # Load UI and initialize all Widgets
        self.loadUI()
        self.assignWidgets()
        self._widget.show()
        context.add_widget(self._widget)


        # for testing only!!!: This parameter should be set for every Robot individually
        rospy.set_param(Name.param_robot_id, "Robot_2")

        # a simple list containing current RobotIDs. The order indicates, which robo gets which Index for
        # tabs and the 4 fields
        self.id_list = []

        # initialize time stamps to identify "old" unuseable udp packages
        self.lastStamps = {LiveTool.UDP_POSITION_MSG: {Name.secs: 0, Name.nsecs: 0},
                           LiveTool.UDP_DETECTION_MSG: {Name.secs: 0, Name.nsecs: 0},
                           LiveTool.UDP_STATUS_MSG: {Name.secs: 0, Name.nsecs: 0},
                           LiveTool.UDP_TRAJECTORY_MSG: {Name.secs: 0, Name.nsecs: 0}}

        # Initialize the UDP-socket for receiving data on a seperate Thread
        self.sock = socket.socket(socket.AF_INET,  # Internet
                                  socket.SOCK_DGRAM)  # UDP
        self.sock.settimeout( LiveTool.UDP_TIMEOUT )
        self.sock.bind((LiveTool.UDP_IP, LiveTool.UDP_PORT))

        # Initialize the UDP-Thread
        self.udp_worker = UdpWorker(self.sock)
        self.udp_worker.signal.connect(self._parseUdpPackage)
        self.udp_worker.start()

        with open("pckgs.txt", "w") as file:
            None
        file.close()


    def receiveIPconfig(self):
        rp = rospkg.RosPack()
        ip_filename = os.path.join(rp.get_path('bitbots_live_tool_rqt'), 'resource', 'ip_config.yaml')

        with open(ip_filename, "r") as file:
            ip_config = yaml.load(file)
            LiveTool.UDP_IP = ip_config.get(LiveTool.RECEIVE_TOKEN)
            LiveTool.UDP_PORT = int(ip_config.get(LiveTool.PORT_TOKEN))
            LiveTool.TIME_OUT = float(ip_config.get(LiveTool.TIMEOUT_TOKEN))
        file.close()
        print ("Receive from: " + str(LiveTool.UDP_IP))
        print ("Receive Timeout: " + str(LiveTool.UDP_TIMEOUT))


    def loadUI(self):
        self._widget = QMainWindow()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('bitbots_live_tool_rqt'), 'resource', LiveTool.UI_FILE)
        loadUi(ui_file, self._widget, {})


    def assignWidgets(self):
        wd = self._widget

        self.single_field = SingleField(wd.fieldFrame, wd.field)


        # assign quartered view, 4 times, and integrate it into 4 defaultviews
        self.quarter_field1 = QuarterField(wd.frm_rob1field, wd.lb_field)
        self.quarter_field2 = QuarterField(wd.frm_rob1field_2, wd.lb_field_2)
        self.quarter_field3 = QuarterField(wd.frm_rob1field_3, wd.lb_field_3)
        self.quarter_field4 = QuarterField(wd.frm_rob1field_4, wd.lb_field_4)

        self.quarter_fields = [self.quarter_field1, self.quarter_field2, self.quarter_field3, self.quarter_field4]

        self.main_window = MainWindow(self.single_field, self.quarter_fields, wd.checkBox, wd.tabWidget, wd.Uebersicht,\
                                      wd.lb_gamephase, wd.lb_gamescore, wd.lb_extratime)

        self.tab1 = InformationTab(0, self.single_field, wd.role1, wd.penalty1, wd.hardwarestatus1, wd.action1, wd.checkHideAll_4, \
                                   wd.checkTarget_4, wd.checkBall_4, wd.checkTeammate_4, wd.checkOpponent_4, wd.checkUndef_4, wd.activity1, wd.activityRed1)
        self.tab2 = InformationTab(1, self.single_field, wd.role2, wd.penalty2, wd.hardwarestatus2, wd.action2, wd.checkHideAll_2, \
                                   wd.checkTarget_2, wd.checkBall_2, wd.checkTeammate_2, wd.checkOpponent_2, wd.checkUndef_2, wd.activity2, wd.activityRed2)
        self.tab3 = InformationTab(2, self.single_field, wd.role3, wd.penalty3, wd.hardwarestatus3, wd.action3, wd.checkHideAll_3, \
                                   wd.checkTarget_3, wd.checkBall_3, wd.checkTeammate_3, wd.checkOpponent_3, wd.checkUndef_3, wd.activity3, wd.activityRed3)
        self.tab4 = InformationTab(3, self.single_field, wd.role4, wd.penalty4, wd.hardwarestatus4, wd.action4, wd.checkHideAll_5, \
                                   wd.checkTarget_5, wd.checkBall_5, wd.checkTeammate_5, wd.checkOpponent_5, wd.checkUndef_5, wd.activity4, wd.activityRed4)


        self.view1 = DefaultView(self.quarter_field1, wd.lb_name, wd.lb_nextaction, wd.lb_penaltytime, \
                 wd.lb_role, wd.lb_statehardware, wd.wdg_stateindicator_green, wd.wdg_stateindicator_red)
        self.view2 = DefaultView(self.quarter_field2, wd.lb_name_2, wd.lb_nextaction_2, wd.lb_penaltytime_3, \
                 wd.lb_role_3, wd.lb_statehardware_3, wd.wdg_stateindicator_green_2, wd.wdg_stateindicator_red_2)
        self.view3 = DefaultView(self.quarter_field3, wd.lb_name_3, wd.lb_nextaction_3, wd.lb_penaltytime_4, \
                 wd.lb_role_4, wd.lb_statehardware_4, wd.wdg_stateindicator_green_3, wd.wdg_stateindicator_red_3)
        self.view4 = DefaultView(self.quarter_field4, wd.lb_name_4, wd.lb_nextaction_4, wd.lb_penaltytime_5, \
                 wd.lb_role_5, wd.lb_statehardware_5, wd.wdg_stateindicator_green_4, wd.wdg_stateindicator_red_4)

    """
    counter = 0
    def count_pkgs(self, p):
        if LiveTool.counter < 256:
            with open("pckgs.txt", "a") as file:
                file.write(p)
            file.close()
            LiveTool.counter += 1
            print(LiveTool.counter)
    """


    def _parseUdpPackage(self, data):
        #self.count_pkgs(str(data))
        robotID, idStr, yamlData = str.split(str(data), LiveTool.UDP_ID_SPLIT)
        dataDict = yaml.load(yamlData)

        print('data: '+ data)
        print('yaml: '+ yamlData)

        # at least a timestamp has to be in message
        if dataDict != {} and dataDict.has_key(Name.timestamp):
            if self.stampIsNew(self.lastStamps.get(idStr), dataDict.get(Name.timestamp)):
                field = self.getID(robotID)

                #print("recieved: " + idStr +"::"+yamlData)
                if idStr == self.UDP_POSITION_MSG:

                    self.single_field.setPositionMsg(robotID, dataDict)

                    if field == 0:
                        self.quarter_field1.setPositionMsg(robotID, dataDict)
                    elif field == 1:
                        self.quarter_field2.setPositionMsg(robotID, dataDict)
                    elif field == 2:
                        self.quarter_field3.setPositionMsg(robotID, dataDict)
                    elif field == 3:
                        self.quarter_field4.setPositionMsg(robotID, dataDict)

                elif idStr == self.UDP_DETECTION_MSG:

                    self.single_field.setDetectionMsg(robotID, dataDict)

                    if field == 0:
                        self.quarter_field1.setDetectionMsg(robotID, dataDict)
                    elif field == 1:
                        self.quarter_field2.setDetectionMsg(robotID, dataDict)
                    elif field == 2:
                        self.quarter_field3.setDetectionMsg(robotID, dataDict)
                    elif field == 3:
                        self.quarter_field4.setDetectionMsg(robotID, dataDict)

                elif idStr == self.UDP_STATUS_MSG:

                    self.main_window.setStatusMsg(dataDict)

                    self.single_field.setStatusMsg(robotID, dataDict)

                    if field == 0:
                        self.tab1.setStatusMsg(dataDict)
                    elif field == 1:
                        self.tab2.setStatusMsg(dataDict)
                    elif field == 2:
                        self.tab3.setStatusMsg(dataDict)
                    elif field == 3:
                        self.tab4.setStatusMsg(dataDict)

                    if field == 0:
                        self.view1.setStatusMsg(robotID, dataDict)
                    elif field == 1:
                        self.view2.setStatusMsg(robotID, dataDict)
                    elif field == 2:
                        self.view3.setStatusMsg(robotID, dataDict)
                    elif field == 3:
                        self.view4.setStatusMsg(robotID, dataDict)

                    if field == 0:
                        self.quarter_field1.setStatusMsg(robotID, dataDict)
                    elif field == 1:
                        self.quarter_field2.setStatusMsg(robotID, dataDict)
                    elif field == 2:
                        self.quarter_field3.setStatusMsg(robotID, dataDict)
                    elif field == 3:
                        self.quarter_field4.setStatusMsg(robotID, dataDict)

                elif idStr == self.UDP_TRAJECTORY_MSG:
                    self.single_field.setTrajectoryMsg(robotID, dataDict)

                    if field == 0:
                        self.quarter_field1.setTrajectoryMsg(robotID, dataDict)
                    elif field == 1:
                        self.quarter_field2.setTrajectoryMsg(robotID, dataDict)
                    elif field == 2:
                        self.quarter_field3.setTrajectoryMsg(robotID, dataDict)
                    elif field == 3:
                        self.quarter_field4.setTrajectoryMsg(robotID, dataDict)

            else:
                print("received old message")


    def getID(self, robotID):
        if not self.id_list.__contains__(robotID):
            self.id_list.append(robotID)

        return self.id_list.index(robotID)


    # returns true if stampNew is bigger or equal to stampOld
    def stampIsNew(self, stampOld, stampNew):
        if stampNew.get(Name.secs) >= stampOld.get(Name.secs):
            if stampNew.get(Name.nsecs) >= stampOld.get(Name.nsecs):
                return True
        return False

    