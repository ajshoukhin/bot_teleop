import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32MultiArray # Added Int32MultiArray
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class BotTeleopNode(Node):

    def __init__(self):
        super().__init__('bot_teleop_node')

        self.joy_subscriber = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10
        )
        self.cmd_vel_publisher = self.create_publisher(
            Twist, '/cmd_vel', 10)
            
        # NEW: Publisher for Flipper commands
        # Format: [Flip1, Flip2, Flip3, Flip4] where value is -1, 0, or 1
        self.flipper_publisher = self.create_publisher(
            Int32MultiArray, '/cmd_flipper', 10)
        
        self.indicator_publisher = self.create_publisher(
            String, '/indicator', 10)

        self.dummyNode = None
        self.send_msg = False
        self.controller_model = None

    def joy_callback(self, msg):
        self.detect_controller_model(msg)
        
        # --- Mode Switching Logic (unchanged) ---
        # xbox-360 
        if self.controller_model ==  "Xbox-360 Controller":
            if msg.buttons[2] == 1 and self.dummyNode is None: # X button
                self.send_msg = True
                self.create_dummy_node()
                self.get_logger().info("Manual Drive")
            elif msg.buttons[1] == 1 and self.dummyNode is not None: # B button
                self.send_msg = False
                self.destroy_dummy_node()
                self.get_logger().info("Autonomous Drive")

        # ps4
        elif self.controller_model ==  "PS4 Controller":
            if msg.buttons[3] == 1 and self.dummyNode is None: # Square
                self.send_msg = True
                self.create_dummy_node()
                self.get_logger().info("Manual Drive")
            elif msg.buttons[1] == 1 and self.dummyNode is not None: # Circle
                self.send_msg = False
                self.destroy_dummy_node()
                self.get_logger().info("Autonomous Drive")

        # logitech x-3D pro (unchanged logic omitted for brevity)
        elif self.controller_model == "Logitech X-3D Pro":
             # ... (keep your existing logitech logic here)
             pass

        # --- Publish Commands ---
        if self.send_msg:
            self.publish_twist_msg(msg)
            self.publish_flipper_msg(msg) # NEW: Call flipper function

    def publish_flipper_msg(self, joy):
        # Array to hold direction for 4 flippers: [F1, F2, F3, F4]
        # 1 = UP, -1 = DOWN, 0 = STOP
        flipper_cmd = Int32MultiArray()
        flipper_cmd.data = [0, 0, 0, 0] 

        if self.controller_model == "PS4 Controller":
            # --- Flipper 1 (Front Left) ---
            if joy.buttons[4]: flipper_cmd.data[1] = -1   # L1 -> Up
            elif joy.axes[2] < -0.5: flipper_cmd.data[1] = 1 # L2 (Trigger) -> Down
            
            # --- Flipper 2 (Front Right) ---
            if joy.buttons[5]: flipper_cmd.data[0] = -1   # R1 -> Up
            elif joy.axes[5] < -0.5: flipper_cmd.data[0] = 1 # R2 (Trigger) -> Down

            # --- Flipper 3 (Rear Left) ---
            # Using D-Pad (Axes 7 is usually vertical D-pad on Linux)
            if joy.axes[7] > 0.5: flipper_cmd.data[3] = 1     # D-Pad Up
            elif joy.axes[7] < -0.5: flipper_cmd.data[3] = -1 # D-Pad Down

            # --- Flipper 4 (Rear Right) ---
            if joy.buttons[2]: flipper_cmd.data[2] = 1    # Triangle -> Up
            elif joy.buttons[0]: flipper_cmd.data[2] = -1 # Cross -> Down

        elif self.controller_model == "Xbox-360 Controller":
            # Similar mapping for Xbox
            if joy.buttons[4]: flipper_cmd.data[0] = 1    # LB
            elif joy.axes[2] < -0.5: flipper_cmd.data[0] = -1 # LT
            
            if joy.buttons[5]: flipper_cmd.data[1] = 1    # RB
            elif joy.axes[5] < -0.5: flipper_cmd.data[1] = -1 # RT

            if joy.axes[7] > 0.5: flipper_cmd.data[2] = -1     # D-Pad Up
            elif joy.axes[7] < -0.5: flipper_cmd.data[2] = 1 # D-Pad Down

            if joy.buttons[3]: flipper_cmd.data[3] = -1    # Y -> Up
            elif joy.buttons[0]: flipper_cmd.data[3] = 1 # A -> Down

        self.flipper_publisher.publish(flipper_cmd)

    # ... (Rest of your existing functions: publish_twist_msg, detect_controller_model, dummy nodes, etc.)
    # Be sure to include the publish_twist_msg logic you wrote!
    def create_dummy_node(self):
        self.dummyNode = rclpy.create_node('teleop_is_on')
        msg = String()
        msg.data = "Blue -> Manual Mode"
        self.indicator_publisher.publish(msg)

    def destroy_dummy_node(self):
        if self.dummyNode is not None:
            self.dummyNode.destroy_node()
            self.dummyNode = None
            msg = String()
            msg.data = "RED -> Autonomous Mode"
            self.indicator_publisher.publish(msg)

    def publish_twist_msg(self, joy):
        twist = Twist()
        # ... (Insert your exact twist logic here) ...
        # (I am omitting the twist logic repetition to save space, paste your block here)
        if self.controller_model ==  "Xbox-360 Controller":
            twist.linear.x = (joy.axes[1]/2) + (joy.axes[4]/2)
            twist.angular.z = (joy.axes[0]/2) + (joy.axes[3]/2)
        elif self.controller_model ==  "PS4 Controller":
            # Right Stick Up/Down for Forward/Backward
            twist.linear.x = joy.axes[4]
            
            # Left Stick Left/Right for Turning
            twist.angular.z = joy.axes[0]
            
            # Apply deadzone [ -0.17, 0.17 ]
            if -0.17 < twist.linear.x < 0.17:
                twist.linear.x = 0.0
                
            if -0.17 < twist.angular.z < 0.17:
                twist.angular.z = 0.0
        
        self.cmd_vel_publisher.publish(twist)

    def detect_controller_model(self, msg):
        if len(msg.buttons) == 11 and len(msg.axes) == 8: self.controller_model = "Xbox-360 Controller"
        elif len(msg.buttons) == 13 and len(msg.axes) == 8: self.controller_model = "PS4 Controller"
        elif len(msg.buttons) == 12 and len(msg.axes) == 6: self.controller_model = "Logitech X-3D Pro"
        else: self.controller_model = "Unknown Controller Model"

def main(args=None):
    rclpy.init(args=args)
    bot_teleop_node = BotTeleopNode()
    try:
        rclpy.spin(bot_teleop_node)
    except KeyboardInterrupt:
        pass
    finally:
        bot_teleop_node.destroy_dummy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
