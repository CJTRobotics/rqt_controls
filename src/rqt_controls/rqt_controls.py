from os import path
from sensor_msgs.msg import ChannelFloat32
import rospy
import rosnode
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QTableWidgetItem
from python_qt_binding.QtGui import QColor

class Controls(Plugin):

    def __init__(self, context):
        super(Controls, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('Controls')

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        _ui_file = path.join(rospkg.RosPack().get_path('rqt_controls'), 'resource', 'Controls.ui')
        # Extend the widget with all attributes and children from UI file
        self._ui = loadUi(_ui_file, self._widget)

        self._ui.ButtonKillnode.clicked.connect(self.onKillnode)

        for i in range(0, 3):
            self._ui.MotorTable.setItem(0, i, QTableWidgetItem())
            self._ui.MotorTable.setItem(1, i, QTableWidgetItem())

        # Give QObjects reasonable names
        self._widget.setObjectName('ControlsUi')
        self._widget.setWindowTitle("Robot Controls")
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        self._ui.BatteryPanel.display(999.9)
        self.motor_stats = rospy.Subscriber("skid_steering_node/motor_stats", ChannelFloat32, self.callback)

    def shutdown_plugin(self):
        self.motor_stats.unregister()

    def onKillnode(self):
        self.motor_stats.unregister()
        rosnode.kill_nodes("skid_steering_node")

    def callback(self, msg):
        battery_voltage = round(msg.values[0], 1)
        self._ui.BatteryPanel.display(battery_voltage)
        self.calc_load(msg.values)

    def calc_load(self, values):
        vel_left = 10*abs(values[1])
        vel_right = 10*abs(values[2])
        i = 0
        for rpm in range (4, 9, 2):
            if vel_right != 0:
                load = abs(values[rpm])/vel_right
                self._ui.MotorTable.item(0, i).setBackground(self.color(load))
            else:
                self._ui.MotorTable.item(0, i).setBackground(QColor(125, 125, 125))
            i = i+1
        i = 0
        for rpm in range (3, 9, 2):
            if vel_left != 0:
                load = abs(values[rpm])/vel_left
                self._ui.MotorTable.item(1, i).setBackground(self.color(load))
            else:
                self._ui.MotorTable.item(1, i).setBackground(QColor(125, 125, 125))
            i = i+1

    def color(self, load):
        return QColor(125/load, 125*load, 0)
