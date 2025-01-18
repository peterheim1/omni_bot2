import wx
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_msgs.msg import String
from sensor_msgs.msg import Joy, JointState, BatteryState
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped
import threading
from collections import deque
import time
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# Define navigation targets as a list of dictionaries
NAVIGATION_TARGETS = [
    {
        "name": "Charging Station",
        "x": 1.0,
        "y": 0.0,
        "orientation": 0.0  # Yaw in degrees
    },
    {
        "name": "Workstation",
        "x": 2.5,
        "y": 3.0,
        "orientation": 90.0
    },
    {
        "name": "Delivery Point",
        "x": -1.5,
        "y": -2.0,
        "orientation": 180.0
    },
    # Add more targets as needed
]

class NavigationClient:
    def __init__(self, node: Node):
        self._node = node
        self._action_client = ActionClient(self._node, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, x: float, y: float, z: float, orientation_quat=None):
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self._node.get_logger().error('NavigateToPose action server not available!')
            wx.CallAfter(self._node.wx_frame.show_message, 'NavigateToPose action server not available!')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self._node.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = z
        
        if orientation_quat:
            goal_msg.pose.pose.orientation.x = orientation_quat[0]
            goal_msg.pose.pose.orientation.y = orientation_quat[1]
            goal_msg.pose.pose.orientation.z = orientation_quat[2]
            goal_msg.pose.pose.orientation.w = orientation_quat[3]
        else:
            goal_msg.pose.pose.orientation.w = 1.0  # Default orientation

        self._node.get_logger().info(f'Sending navigation goal: x={x}, y={y}, z={z}, orientation={orientation_quat}')
        wx.CallAfter(self._node.wx_frame.show_message, f'Sending navigation goal to ({x}, {y}, {z})')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._node.get_logger().error('Navigation goal rejected!')
            wx.CallAfter(self._node.wx_frame.show_message, 'Navigation goal rejected!')
            return

        self._node.get_logger().info('Navigation goal accepted.')
        wx.CallAfter(self._node.wx_frame.show_message, 'Navigation goal accepted.')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status

        if status == 4:
            self._node.get_logger().info('Navigation goal succeeded!')
            wx.CallAfter(self._node.wx_frame.show_message, 'Navigation goal succeeded!')
        elif status == 5:
            self._node.get_logger().info('Navigation goal was canceled!')
            wx.CallAfter(self._node.wx_frame.show_message, 'Navigation goal was canceled!')
        elif status == 2:
            self._node.get_logger().info('Navigation goal failed!')
            wx.CallAfter(self._node.wx_frame.show_message, 'Navigation goal failed!')
        else:
            self._node.get_logger().info(f'Navigation ended with status code: {status}')
            wx.CallAfter(self._node.wx_frame.show_message, f'Navigation ended with status code: {status}')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        current_pose = feedback.current_pose
        self._node.get_logger().debug(f'Current pose: x={current_pose.pose.position.x}, y={current_pose.pose.position.y}')
        wx.CallAfter(
            self._node.wx_frame.update_odom_info,
            current_pose.pose.position.x,
            current_pose.pose.position.y,
            0.0,  # Assuming 2D navigation
            0.0, 0.0, 0.0  # Placeholder for orientation
        )

class BatteryMonitor(Node):
    MIN_VOLTAGE = 11.0  # Example minimum voltage for 0%
    MAX_VOLTAGE = 14.8  # Example maximum voltage for 100%

    def __init__(self, wx_frame):
        super().__init__('battery_monitor_gui')

        self.get_logger().info("BatteryMonitor node initialized.")

        self.wx_frame = wx_frame

        # Smoothing parameters
        self.smoothing_window = 10  # Number of readings to average
        self.battery_readings = deque(maxlen=self.smoothing_window)

        # Store the latest power_supply_status
        self.power_supply_status = 0  # Default to UNKNOWN

        # Debounce parameters
        self.last_button_press_time = 0
        self.debounce_time = 0.5  # 500 ms debounce time

        # QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers
        self.battery_subscriber = self.create_subscription(
            BatteryState,
            '/battery_state',
            self.battery_callback,
            qos_profile
        )
        self.joy_subscriber = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            qos_profile
        )
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            qos_profile
        )
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile
        )

        # Service clients
        self.service_client = self.create_client(Empty, '/cal')
        if not self.service_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service '/cal' not available after waiting.")

        self.dock_service_client = self.create_client(Empty, 'start_docking')
        if not self.dock_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("'start_docking' service not available, waiting...")

        self.undock_service_client = self.create_client(Empty, 'undock')
        if not self.undock_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("'undock' service not available, waiting...")

        # Publishers
        self.text_publisher = self.create_publisher(String, '/text', qos_profile)
        self.navigate_publisher = self.create_publisher(PoseStamped, '/navigate', qos_profile)

        # Initialize Navigation Client
        self.navigation_client = NavigationClient(self)

        # Timer to update battery display periodically
        self.create_timer(5.0, self.update_battery_display)

    def battery_callback(self, msg):
        # Add the new battery voltage reading to the deque for smoothing
        self.battery_readings.append(msg.voltage)
        self.power_supply_status = msg.power_supply_status
        self.get_logger().debug(f"Received battery voltage: {msg.voltage} V, Status: {msg.power_supply_status}")
        # Update the docked status in the GUI
        wx.CallAfter(self.wx_frame.update_docked_status, self.get_docked_status())

    def get_docked_status(self):
        """
        Determine the docking status based on power_supply_status.
        1 corresponds to POWER_SUPPLY_STATUS_CHARGING which we map to 'Docked'.
        Any other value maps to 'Undocked'.
        """
        if self.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING:
            return "Docked"
        else:
            return "Undocked"

    def joy_callback(self, msg):
        # Check if button 1 is pressed
        if len(msg.buttons) > 1:
            current_time = time.time()
            if msg.buttons[1] == 1 and (current_time - self.last_button_press_time) > self.debounce_time:
                self.last_button_press_time = current_time
                if not self.service_client.wait_for_service(timeout_sec=1.0):
                    self.get_logger().warn("Service '/cal' not available.")
                    wx.CallAfter(self.wx_frame.show_message, "Service '/cal' not available.")
                    return

                request = Empty.Request()
                future = self.service_client.call_async(request)
                future.add_done_callback(self.service_response_callback)
        else:
            self.get_logger().warn("Received Joy message with insufficient buttons.")
            wx.CallAfter(self.wx_frame.show_message, "Joystick message has insufficient buttons.")

    def service_response_callback(self, future):
        try:
            future.result()
            # Notify success via logging only
            self.get_logger().info("Service '/cal' called successfully.")
            # Pop-up message removed as per user request
        except Exception as e:
            # Notify failure via logging only
            self.get_logger().error(f"Service '/cal' call failed: {e}")
            # Pop-up message removed as per user request

    def update_battery_display(self):
        # Calculate the smoothed battery voltage
        if self.battery_readings:
            smoothed_voltage = sum(self.battery_readings) / len(self.battery_readings)
            battery_percentage = self.voltage_to_percentage(smoothed_voltage)
            wx.CallAfter(self.wx_frame.battery_label.SetLabel, 
                        f"Battery State: {battery_percentage:.2f}% ({smoothed_voltage:.2f} V)")
            self.get_logger().debug(f"Updated battery display: {battery_percentage:.2f}% ({smoothed_voltage:.2f} V)")
        else:
            wx.CallAfter(self.wx_frame.battery_label.SetLabel, "Battery State: N/A")

    def voltage_to_percentage(self, voltage):
        if voltage <= self.MIN_VOLTAGE:
            return 0.0
        elif voltage >= self.MAX_VOLTAGE:
            return 100.0
        else:
            return ((voltage - self.MIN_VOLTAGE) / (self.MAX_VOLTAGE - self.MIN_VOLTAGE)) * 100.0

    def publish_text_message(self, message):
        if message.strip():
            # Publish the string message on the /text topic
            msg = String()
            msg.data = message
            self.text_publisher.publish(msg)
            self.get_logger().info(f"Published message: {message}")
        else:
            self.get_logger().warn("Attempted to publish an empty message.")
            wx.CallAfter(self.wx_frame.show_message, "Cannot send empty message.")

    def joint_state_callback(self, msg):
        # Assuming positions and velocities are ordered as S0-S3 and V0-V3
        try:
            # Extract first four positions (S0-S3) and convert to degrees
            s_values = [math.degrees(s) for s in msg.position[:4]]
            # Extract velocities for V0-V3 and scale by 100
            v_values = [v * 100 for v in msg.velocity[:4]]  # Corrected indexing
                
            # Update the GUI with wheel information
            wx.CallAfter(
                self.wx_frame.update_wheel_info,
                s_values,
                v_values
            )
            self.get_logger().debug(f"Updated wheel info: S={s_values}, V={v_values}")
        except IndexError as e:
            self.get_logger().warn(f"Insufficient joint state data: {e}")

    def odom_callback(self, msg):
        # Extract position
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        
        # Convert quaternion to Euler angles for readability
        roll, pitch, yaw = self.quaternion_to_euler(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        )
        
        # Update the GUI with odometry information
        wx.CallAfter(
            self.wx_frame.update_odom_info,
            position.x,
            position.y,
            position.z,
            math.degrees(roll),
            math.degrees(pitch),
            math.degrees(yaw)
        )
        self.get_logger().debug(f"Updated odometry info: Position=({position.x}, {position.y}, {position.z}), "
                                f"Orientation=({math.degrees(roll)}, {math.degrees(pitch)}, {math.degrees(yaw)})")

    @staticmethod
    def quaternion_to_euler(x, y, z, w):
        """
        Convert a quaternion into Euler angles (roll, pitch, yaw)
        """
        # Reference: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        
        return roll, pitch, yaw

    # Docking Methods
    def dock_robot(self):
        """Call the 'start_docking' service."""
        if not self.dock_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("'start_docking' service not available!")
            wx.CallAfter(self.wx_frame.show_message, "'start_docking' service not available!")
            return

        request = Empty.Request()
        self.get_logger().info("Calling 'start_docking' service...")
        wx.CallAfter(self.wx_frame.show_message, "Calling 'start_docking' service...")

        future = self.dock_service_client.call_async(request)
        future.add_done_callback(self.dock_service_callback)

    def undock_robot(self):
        """Call the 'undock' service."""
        if not self.undock_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("'undock' service not available!")
            wx.CallAfter(self.wx_frame.show_message, "'undock' service not available!")
            return

        request = Empty.Request()
        self.get_logger().info("Calling 'undock' service...")
        wx.CallAfter(self.wx_frame.show_message, "Calling 'undock' service...")

        future = self.undock_service_client.call_async(request)
        future.add_done_callback(self.undock_service_callback)

    def dock_service_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info("'start_docking' service call succeeded.")
            wx.CallAfter(self.wx_frame.show_message, "'start_docking' service call succeeded.")
        except Exception as e:
            self.get_logger().error(f"'start_docking' service call failed: {e}")
            wx.CallAfter(self.wx_frame.show_message, f"'start_docking' service call failed: {e}")

    def undock_service_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info("'undock' service call succeeded.")
            wx.CallAfter(self.wx_frame.show_message, "'undock' service call succeeded.")
        except Exception as e:
            self.get_logger().error(f"'undock' service call failed: {e}")
            wx.CallAfter(self.wx_frame.show_message, f"'undock' service call failed: {e}")

    def navigate_robot_to_destination(self, x, y, yaw_rad):
        """
        Navigate the robot to the specified (x, y) coordinates with the given yaw orientation.
        """
        # Convert yaw angle to quaternion
        q = self.euler_to_quaternion(0.0, 0.0, yaw_rad)
        
        self.get_logger().info(f"Navigating to ({x}, {y}) with yaw {math.degrees(yaw_rad)} degrees.")
        self.navigation_client.send_goal(x, y, 0.0, orientation_quat=q)

    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        """
        Convert Euler angles to a quaternion.
        """
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return (qx, qy, qz, qw)

class WheelPanel(wx.Panel):
    def __init__(self, parent):
        super().__init__(parent, size=(300, 300))  # Increased size for better visibility
        self.parent = parent
        self.Bind(wx.EVT_PAINT, self.on_paint)
        
        # Initialize wheel positions and velocities
        self.s_values = ['N/A'] * 4  # S0-S3
        self.v_values = [0.0] * 4    # V0-V3

    def update_wheel_data(self, s_values, v_values):
        # Update the internal state
        self.s_values = [f"{s:.2f}°" for s in s_values]
        self.v_values = v_values
        # Trigger a repaint
        self.Refresh()

    def on_paint(self, event):
        dc = wx.PaintDC(self)
        dc.Clear()
        
        # Get the size of the panel
        width, height = self.GetSize()
        
        # Double font size
        font = self.GetFont()
        font.SetPointSize(font.GetPointSize() * 2)  # Double the current font size
        dc.SetFont(font)
        
        # Define positions for S0, S1, S2, S3
        margin = 40
        label_offset = 50  # Increased space between the S and V labels to account for font size
        
        # Top row: S0, S3 labels with V0 and V3 below them
        s0_pos = (margin, margin)
        s3_pos = (width - margin - 100, margin)  # Adjusted for larger font size
        
        # Bottom row: S1, S2 labels with V1 and V2 below them
        s1_pos = (margin, height - margin - 100)  # Adjusted for larger font size
        s2_pos = (width - margin - 100, height - margin - 100)
        
        # Set colors
        red_color = wx.Colour(255, 0, 0)
        black_color = wx.Colour(0, 0, 0)
        
        # Draw S0 and V0
        self.draw_label(dc, f"S0: {self.s_values[0]}", s0_pos[0], s0_pos[1], self.get_float_value(self.s_values[0]), red_color, black_color)
        self.draw_label(dc, f"V0: {self.v_values[0]:.2f}", s0_pos[0], s0_pos[1] + label_offset, self.v_values[0], red_color, black_color)
        
        # Draw S3 and V3
        self.draw_label(dc, f"S3: {self.s_values[3]}", s3_pos[0], s3_pos[1], self.get_float_value(self.s_values[3]), red_color, black_color)
        self.draw_label(dc, f"V3: {self.v_values[3]:.2f}", s3_pos[0], s3_pos[1] + label_offset, self.v_values[3], red_color, black_color)
        
        # Draw S1 and V1
        self.draw_label(dc, f"S1: {self.s_values[1]}", s1_pos[0], s1_pos[1], self.get_float_value(self.s_values[1]), red_color, black_color)
        self.draw_label(dc, f"V1: {self.v_values[1]:.2f}", s1_pos[0], s1_pos[1] + label_offset, self.v_values[1], red_color, black_color)
        
        # Draw S2 and V2
        self.draw_label(dc, f"S2: {self.s_values[2]}", s2_pos[0], s2_pos[1], self.get_float_value(self.s_values[2]), red_color, black_color)
        self.draw_label(dc, f"V2: {self.v_values[2]:.2f}", s2_pos[0], s2_pos[1] + label_offset, self.v_values[2], red_color, black_color)

    def draw_label(self, dc, text, x, y, value, red_color, black_color):
        """
        Draws the label with the appropriate color based on the value.
        If the value is negative, it's drawn in red;
        """
        if isinstance(value, float) and value < 0:
            dc.SetTextForeground(red_color)
        else:
            dc.SetTextForeground(black_color)
        dc.DrawText(text, x, y)
    
    @staticmethod
    def get_float_value(text):
        try:
            return float(text.split(':')[1].replace('°', '').strip())
        except (IndexError, ValueError):
            return 0.0

class MyApp(wx.App):
    def OnInit(self):
        self.frame = wx.Frame(None, title='Robot GUI', size=(800, 800))  # Increased size for better layout
        panel = wx.Panel(self.frame)
        vbox = wx.BoxSizer(wx.VERTICAL)
        
        # Battery State
        self.battery_label = wx.StaticText(panel, label="Battery State: N/A", style=wx.ALIGN_CENTER)
        font = self.battery_label.GetFont()
        font.PointSize += 4
        font = font.Bold()
        self.battery_label.SetFont(font)
        vbox.Add(self.battery_label, flag=wx.ALIGN_CENTER | wx.TOP, border=10)
        
        # Docked Status
        self.docked_label = wx.StaticText(panel, label="Docked Status: N/A", style=wx.ALIGN_CENTER)
        font_ds = self.docked_label.GetFont()
        font_ds.PointSize += 2
        font_ds = font_ds.Bold()
        self.docked_label.SetFont(font_ds)
        vbox.Add(self.docked_label, flag=wx.ALIGN_CENTER | wx.TOP, border=5)
        
        # Navigation Targets Selection
        nav_targets_box = wx.StaticBox(panel, label="Navigation Targets")
        nav_targets_sizer = wx.StaticBoxSizer(nav_targets_box, wx.HORIZONTAL)
        
        # Drop-down (ComboBox) for navigation targets
        self.nav_choice = wx.ComboBox(
            panel,
            choices=[target["name"] for target in NAVIGATION_TARGETS],
            style=wx.CB_READONLY
        )
        self.nav_choice.SetSelection(0)  # Select the first target by default
        
        # "Go" button
        self.go_button = wx.Button(panel, label="Go")
        
        nav_targets_sizer.Add(self.nav_choice, proportion=1, flag=wx.EXPAND | wx.ALL, border=5)
        nav_targets_sizer.Add(self.go_button, flag=wx.EXPAND | wx.ALL, border=5)
        
        vbox.Add(nav_targets_sizer, flag=wx.EXPAND | wx.LEFT | wx.RIGHT | wx.TOP, border=10)
        
        # Wheel Information with Graphical Arrows and Velocity Labels
        wheel_box = wx.StaticBox(panel, label="Wheel Information")
        wheel_sizer = wx.StaticBoxSizer(wheel_box, wx.VERTICAL)
        
        # Create WheelPanel for graphical representation
        self.wheel_panel = WheelPanel(panel)
        wheel_sizer.Add(self.wheel_panel, proportion=1, flag=wx.EXPAND | wx.ALL, border=10)
        
        vbox.Add(wheel_sizer, flag=wx.EXPAND | wx.LEFT | wx.RIGHT | wx.TOP, border=10)
        
        # Odometry Information
        odom_box = wx.StaticBox(panel, label="Odometry Information")
        odom_sizer = wx.StaticBoxSizer(odom_box, wx.VERTICAL)
        
        self.odom_position_label = wx.StaticText(panel, label="Position - x: N/A, y: N/A, z: N/A")
        self.odom_pose_label = wx.StaticText(panel, label="Pose - Roll: N/A°, Pitch: N/A°, Yaw: N/A°")
        
        odom_sizer.Add(self.odom_position_label, flag=wx.LEFT | wx.TOP, border=5)
        odom_sizer.Add(self.odom_pose_label, flag=wx.LEFT | wx.TOP, border=5)
        
        vbox.Add(odom_sizer, flag=wx.EXPAND | wx.LEFT | wx.RIGHT | wx.TOP, border=10)
        
        # Text input and send button
        hbox = wx.BoxSizer(wx.HORIZONTAL)
        self.text_input = wx.TextCtrl(panel, style=wx.TE_PROCESS_ENTER)
        self.send_button = wx.Button(panel, label="Send")
        
        hbox.Add(self.text_input, proportion=1, flag=wx.EXPAND | wx.RIGHT, border=5)
        hbox.Add(self.send_button, flag=wx.EXPAND)
        
        vbox.Add(hbox, flag=wx.EXPAND | wx.LEFT | wx.RIGHT | wx.TOP, border=10)
        
        # Dock and Undock buttons
        dock_hbox = wx.BoxSizer(wx.HORIZONTAL)
        self.dock_button = wx.Button(panel, label="Dock")
        self.undock_button = wx.Button(panel, label="Undock")
        
        dock_hbox.Add(self.dock_button, flag=wx.EXPAND | wx.RIGHT, border=5)
        dock_hbox.Add(self.undock_button, flag=wx.EXPAND)
        
        vbox.Add(dock_hbox, flag=wx.ALIGN_CENTER | wx.LEFT | wx.RIGHT | wx.TOP, border=10)
        
        # Navigate button
        navigate_hbox = wx.BoxSizer(wx.HORIZONTAL)
        self.navigate_button = wx.Button(panel, label="Navigate to (1, 0, 0)")
        
        navigate_hbox.Add(self.navigate_button, flag=wx.EXPAND)
        
        vbox.Add(navigate_hbox, flag=wx.ALIGN_CENTER | wx.LEFT | wx.RIGHT | wx.TOP, border=10)
        
        # Add a quit button
        self.quit_button = wx.Button(panel, label="Quit")
        vbox.Add(self.quit_button, flag=wx.ALIGN_RIGHT | wx.ALL, border=10)
        
        panel.SetSizer(vbox)
        
        # Bind the send button to the publish method
        self.send_button.Bind(wx.EVT_BUTTON, self.on_send)
        
        # Bind the Enter key to the text input box
        self.text_input.Bind(wx.EVT_TEXT_ENTER, self.on_send)
        
        # Bind the dock and undock buttons
        self.dock_button.Bind(wx.EVT_BUTTON, self.on_dock)
        self.undock_button.Bind(wx.EVT_BUTTON, self.on_undock)
        
        # Bind the navigate button
        self.navigate_button.Bind(wx.EVT_BUTTON, self.on_navigate)
        
        # Bind the "Go" button
        self.go_button.Bind(wx.EVT_BUTTON, self.on_go)
        
        # Bind the quit button
        self.quit_button.Bind(wx.EVT_BUTTON, self.on_quit)
        
        # Bind close event to handle shutdown
        self.frame.Bind(wx.EVT_CLOSE, self.on_close)
        
        self.frame.Show(True)
        
        # Initialize ROS 2 node
        self.node = BatteryMonitor(self)
        
        # Start ROS 2 spinning in a separate thread
        self.ros_thread = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        self.ros_thread.start()
        
        # Use the node's logger instead of self.get_logger()
        self.node.get_logger().info("GUI initialized and ROS node started.")
        
        return True

    def OnExit(self):
        # Shutdown ROS 2 node and stop spinning when the application is closed
        if rclpy.ok():
            self.node.destroy_node()
            rclpy.shutdown()
        self.ros_thread.join()  # Ensure the ROS thread has finished
        return 0

    def on_send(self, event):
        # Get the text from the input box and send it
        message = self.text_input.GetValue()
        self.node.publish_text_message(message)
        self.text_input.Clear()  # Clear the input box after sending

    def on_dock(self, event):
        # Call the dock_robot method in the node
        self.node.dock_robot()

    def on_undock(self, event):
        # Call the undock_robot method in the node
        self.node.undock_robot()
    
    def on_navigate(self, event):
        # Call the navigate_robot_to_destination method in the node to navigate to (1, 0, 0)
        self.node.navigate_robot_to_destination(1.0, 0.0, 0.0)

    def on_go(self, event):
        selected_index = self.nav_choice.GetSelection()
        if selected_index == wx.NOT_FOUND:
            self.show_message("No navigation target selected.")
            return
        
        target = NAVIGATION_TARGETS[selected_index]
        x = target["x"]
        y = target["y"]
        orientation_deg = target["orientation"]
        
        # Convert orientation from degrees to radians for ROS
        orientation_rad = math.radians(orientation_deg)
        
        # Send navigation goal
        self.node.navigate_robot_to_destination(x, y, orientation_rad)
    
    def on_quit(self, event):
        self.frame.Close(True)  # Close the frame, which will trigger on_close

    def on_close(self, event):
        self.ExitMainLoop()  # Use ExitMainLoop to terminate the main loop gracefully
        event.Skip()  # Ensure the window closes

    def show_message(self, message):
        wx.MessageBox(message, "Information", wx.OK | wx.ICON_INFORMATION)

    # Methods to update wheel and odometry information
    def update_wheel_info(self, s_values, v_values):
        self.wheel_panel.update_wheel_data(s_values, v_values)

    def update_odom_info(self, x, y, z, roll, pitch, yaw):
        self.odom_position_label.SetLabel(
            f"Position - x: {x:.2f} m, y: {y:.2f} m, z: {z:.2f} m"
        )
        self.odom_pose_label.SetLabel(
            f"Pose - Roll: {roll:.2f}°, Pitch: {pitch:.2f}°, Yaw: {yaw:.2f}°"
        )

    def update_docked_status(self, status):
        self.docked_label.SetLabel(f"Docked Status: {status}")

class WheelPanel(wx.Panel):
    def __init__(self, parent):
        super().__init__(parent, size=(300, 300))  # Increased size for better visibility
        self.parent = parent
        self.Bind(wx.EVT_PAINT, self.on_paint)
        
        # Initialize wheel positions and velocities
        self.s_values = ['N/A'] * 4  # S0-S3
        self.v_values = [0.0] * 4    # V0-V3

    def update_wheel_data(self, s_values, v_values):
        # Update the internal state
        self.s_values = [f"{s:.2f}°" for s in s_values]
        self.v_values = v_values
        # Trigger a repaint
        self.Refresh()

    def on_paint(self, event):
        dc = wx.PaintDC(self)
        dc.Clear()
        
        # Get the size of the panel
        width, height = self.GetSize()
        
        # Double font size
        font = self.GetFont()
        font.SetPointSize(font.GetPointSize() * 2)  # Double the current font size
        dc.SetFont(font)
        
        # Define positions for S0, S1, S2, S3
        margin = 40
        label_offset = 50  # Increased space between the S and V labels to account for font size
        
        # Top row: S0, S3 labels with V0 and V3 below them
        s0_pos = (margin, margin)
        s3_pos = (width - margin - 100, margin)  # Adjusted for larger font size
        
        # Bottom row: S1, S2 labels with V1 and V2 below them
        s1_pos = (margin, height - margin - 100)  # Adjusted for larger font size
        s2_pos = (width - margin - 100, height - margin - 100)
        
        # Set colors
        red_color = wx.Colour(255, 0, 0)
        black_color = wx.Colour(0, 0, 0)
        
        # Draw S0 and V0
        self.draw_label(dc, f"S0: {self.s_values[0]}", s0_pos[0], s0_pos[1], self.get_float_value(self.s_values[0]), red_color, black_color)
        self.draw_label(dc, f"V0: {self.v_values[0]:.2f}", s0_pos[0], s0_pos[1] + label_offset, self.v_values[0], red_color, black_color)
        
        # Draw S3 and V3
        self.draw_label(dc, f"S3: {self.s_values[3]}", s3_pos[0], s3_pos[1], self.get_float_value(self.s_values[3]), red_color, black_color)
        self.draw_label(dc, f"V3: {self.v_values[3]:.2f}", s3_pos[0], s3_pos[1] + label_offset, self.v_values[3], red_color, black_color)
        
        # Draw S1 and V1
        self.draw_label(dc, f"S1: {self.s_values[1]}", s1_pos[0], s1_pos[1], self.get_float_value(self.s_values[1]), red_color, black_color)
        self.draw_label(dc, f"V1: {self.v_values[1]:.2f}", s1_pos[0], s1_pos[1] + label_offset, self.v_values[1], red_color, black_color)
        
        # Draw S2 and V2
        self.draw_label(dc, f"S2: {self.s_values[2]}", s2_pos[0], s2_pos[1], self.get_float_value(self.s_values[2]), red_color, black_color)
        self.draw_label(dc, f"V2: {self.v_values[2]:.2f}", s2_pos[0], s2_pos[1] + label_offset, self.v_values[2], red_color, black_color)

    def draw_label(self, dc, text, x, y, value, red_color, black_color):
        """
        Draws the label with the appropriate color based on the value.
        If the value is negative, it's drawn in red;
        """
        if isinstance(value, float) and value < 0:
            dc.SetTextForeground(red_color)
        else:
            dc.SetTextForeground(black_color)
        dc.DrawText(text, x, y)
    
    @staticmethod
    def get_float_value(text):
        try:
            return float(text.split(':')[1].replace('°', '').strip())
        except (IndexError, ValueError):
            return 0.0

if __name__ == '__main__':
    try:
        # Initialize ROS 2
        rclpy.init()
    except Exception as e:
        print(f"Failed to initialize ROS 2: {e}")
        exit(1)

    try:
        # Initialize wxPython
        app = MyApp(False)
        
        # Run wxPython main loop
        app.MainLoop()
    except Exception as e:
        print(f"An error occurred while running the GUI: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()
