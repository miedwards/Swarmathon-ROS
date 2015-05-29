# TODO: insert header here

# TODO: remove old commented out code to clean it up

from __future__ import division
import os
import rospkg

# TODO: remove unused imports
from std_msgs.msg import Int32
from std_msgs.msg import UInt64
from std_msgs.msg import Float32
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
import rospy
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Slot
from python_qt_binding.QtGui import QKeySequence, QShortcut, QWidget
from rqt_gui_py.plugin import Plugin


class RoverDriverRqtMonitor(Plugin):
  
    # initialization routine
    def __init__(self, context):
        super(RoverDriverRqtMonitor, self).__init__(context)
        self.setObjectName('RoverDriverRqtMonitor')

	# load the user interface
        self._widget = QWidget()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rover_driver_rqt_monitor'), 'resource', 'RoverDriverRqtMonitor.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('RoverDriverRqtMonitorUi')
	# TODO: Look into this serial number feature.  Might want to use it.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

	# set up callback routines to get called when data changes
	self._subscriber = rospy.Subscriber("/moe/state_machine", String, self.Moe_State_callback)
	self._subscriber = rospy.Subscriber("/moe/harvested", UInt64, self.Moe_Targets_callback)
	self._subscriber = rospy.Subscriber("/moe/location_real", Pose2D, self.Moe_Position_callback)

	self._subscriber = rospy.Subscriber("/larry/state_machine", String, self.Larry_State_callback)
	self._subscriber = rospy.Subscriber("/larry/harvested", UInt64, self.Larry_Targets_callback)
	self._subscriber = rospy.Subscriber("/larry/location_real", Pose2D, self.Larry_Position_callback)

	self._subscriber = rospy.Subscriber("/curly/state_machine", String, self.Curly_State_callback)
	self._subscriber = rospy.Subscriber("/curly/harvested", UInt64, self.Curly_Targets_callback)
	self._subscriber = rospy.Subscriber("/curly/location_real", Pose2D, self.Curly_Position_callback)

	self._subscriber = rospy.Subscriber("/shemp/state_machine", String, self.Shemp_State_callback)
	self._subscriber = rospy.Subscriber("/shemp/harvested", UInt64, self.Shemp_Targets_callback)
	self._subscriber = rospy.Subscriber("/shemp/location_real", Pose2D, self.Shemp_Position_callback)
      
    def Moe_State_callback(self, message):
	    self._widget.MoeStateData.setText('%s' % (message.data))

    def Moe_Targets_callback(self, message):
	    self._widget.MoeTargetData.setText('%d' % (message.data))

    def Moe_Position_callback(self, message):
	    self._widget.MoeXPositionData.setText('%.1f, ' % (message.x))
	    self._widget.MoeYPositionData.setText('%.1f ' % (message.y))
	    self._widget.MoeHeadingData.setText('(%.1f)' % (message.theta))

    def Larry_State_callback(self, message):
	    self._widget.LarryStateData.setText('%s' % (message.data))

    def Larry_Targets_callback(self, message):
	    self._widget.LarryTargetData.setText('%d' % (message.data))

    def Larry_Position_callback(self, message):
	    self._widget.LarryXPositionData.setText('%.1f, ' % (message.x))
	    self._widget.LarryYPositionData.setText('%.1f ' % (message.y))
	    self._widget.LarryHeadingData.setText('(%.1f)' % (message.theta))

    def Curly_State_callback(self, message):
	    self._widget.CurlyStateData.setText('%s' % (message.data))

    def Curly_Targets_callback(self, message):
	    self._widget.CurlyTargetData.setText('%d' % (message.data))

    def Curly_Position_callback(self, message):
	    self._widget.CurlyXPositionData.setText('%.1f, ' % (message.x))
	    self._widget.CurlyYPositionData.setText('%.1f ' % (message.y))
	    self._widget.CurlyHeadingData.setText('(%.1f)' % (message.theta))

    def Shemp_State_callback(self, message):
	    self._widget.ShempStateData.setText('%s' % (message.data))

    def Shemp_Targets_callback(self, message):
	    self._widget.ShempTargetData.setText('%d' % (message.data))

    def Shemp_Position_callback(self, message):
	    self._widget.ShempXPositionData.setText('%.1f, ' % (message.x))
	    self._widget.ShempYPositionData.setText('%.1f ' % (message.y))
	    self._widget.ShempHeadingData.setText('(%.1f)' % (message.theta))



