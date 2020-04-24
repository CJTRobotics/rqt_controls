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
        self.display_voltage(msg.values[0])
        rpm = msg.values[3:9]
        #Left wheels
        self.set_color([rpm[0], rpm[2], rpm[4]], msg.values[1], 1)
        #Right wheels
        self.set_color([rpm[1], rpm[3], rpm[5]], msg.values[2], 0)
        for i in range(0, 6):
            if i%2 == 0:
                #Left wheels
                self._ui.MotorTable.item(1, i/2).setText("%2d" % rpm[i])
            else:
                #Right wheels.setData(rpm[i]
                self._ui.MotorTable.item(0, i//2).setText("%2d" % rpm[i])

    def color(self, vel, rpm):
        if rpm != 0:
            load = vel/rpm
        else:
            load = 100
        print("Load: " + str(load) + " VEL: " + str(vel) + " RPM: " + str(rpm) + "\n")
        if load > 1.5:
            return QColor(255, 0, 0)
        elif load > 1.3:
            #Dunkelorange
            return QColor(255, 100, 0)
        elif load > 1.1:
            #Hellorange
            return QColor(240, 165, 0)
        elif load >= 0:
            return QColor(0, 125, 0)
        else:
            #Negative Werte -> Rot
            return QColor(225, 0, 0)

    def set_color(self, rpms, vel, row):
        if vel != 0:
            vel *= 100
            for i in range(0, 3):
                self._ui.MotorTable.item(row, i).setBackground(self.color(vel, rpms[i]))
        else:
            for i in range(0, 3):
                self._ui.MotorTable.item(row, i).setBackground(QColor(125, 125, 125))

    def display_voltage(self, battery_voltage):
        self._ui.BatteryPanel.display("%.1f" % battery_voltage)