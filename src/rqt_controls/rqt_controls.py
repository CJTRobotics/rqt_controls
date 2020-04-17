from os import path
from sensor_msgs.msg import ChannelFloat32
import rospy
import rosnode
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

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
        battery_voltage = round(msg.values[0], 2)
        self._ui.BatteryPanel.display(battery_voltage)
