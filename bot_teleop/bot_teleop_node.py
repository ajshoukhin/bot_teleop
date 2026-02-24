import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32MultiArray, Float64MultiArray, Int32, Bool
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

# --- Helper Functions for Arm Control ---
def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def dz(x, deadzone):
    if abs(x) < deadzone:
        return 0.0
    s = 1.0 if x >= 0 else -1.0
    x = (abs(x) - deadzone) / (1.0 - deadzone)
    return s * clamp(x, 0.0, 1.0)

def expo(x, e):
    s = 1.0 if x >= 0 else -1.0
    return s * (abs(x) ** e)

class BotTeleopNode(Node):

    def __init__(self):
        super().__init__('bot_teleop_node')

        # --- Subscribers ---
        self.joy_subscriber = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # --- Drive Publishers ---
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.flipper_publisher = self.create_publisher(Int32MultiArray, '/cmd_flipper', 10)
        
        # --- Arm Publishers ---
        self.arm_vel_pub = self.create_publisher(Float64MultiArray, '/arm/cmd_vel', 10)
        self.arm_grip_pub = self.create_publisher(Int32, '/arm/cmd_grip', 10)
        self.arm_enable_pub = self.create_publisher(Bool, '/arm/enable', 10)

        # --- Indicator Publisher ---
        self.indicator_publisher = self.create_publisher(String, '/indicator', 10)

        # --- Internal State ---
        self.dummyNode = None
        self.send_msg = False          # True = Manual Control Active
        self.controller_model = None
        
        self.control_mode = "DRIVE"    # Options: "DRIVE", "ARM"
        self.arm_enabled = False       # Tracks if we have sent the enable signal to the arm

        # Arm Config
        self.ARM_DEADZONE = 0.10
        self.ARM_EXPO = 1.6
        self.ARM_MAX_VEL = [0.3, 0.3, 0.3, 0.4, 0.4]
        self.ARM_YAW_SPEED = 0.2

    def joy_callback(self, msg):
        self.detect_controller_model(msg)
        
        # ---------------------------------------------------------
        # 1. Master Switch: Manual (Teleop) vs Autonomous
        # ---------------------------------------------------------
        if self.controller_model == "PS4 Controller":
            # Square (3) -> Engage Manual Control
            if msg.buttons[3] == 1 and self.dummyNode is None:
                self.send_msg = True
                self.create_dummy_node()
                self.get_logger().info("MANUAL CONTROL ENGAGED")
            
            # Circle (1) -> Disengage (Go to Autonomous)
            elif msg.buttons[1] == 1 and self.dummyNode is not None:
                self.send_msg = False
                self.destroy_dummy_node()
                self.get_logger().info("AUTONOMOUS MODE ENGAGED")

        elif self.controller_model == "Xbox-360 Controller":
            # X (2) -> Manual, B (1) -> Auto
            if msg.buttons[2] == 1 and self.dummyNode is None:
                self.send_msg = True
                self.create_dummy_node()
                self.get_logger().info("MANUAL CONTROL ENGAGED")
            elif msg.buttons[1] == 1 and self.dummyNode is not None:
                self.send_msg = False
                self.destroy_dummy_node()
                self.get_logger().info("AUTONOMOUS MODE ENGAGED")

        # ---------------------------------------------------------
        # 2. Sub-Mode Switch: Drive vs Arm (Only if Manual is ON)
        # ---------------------------------------------------------
        if self.send_msg:
            if self.controller_model == "PS4 Controller":
                # OPTIONS (Button 9) -> Drive Mode
                if msg.buttons[9] == 1 and self.control_mode != "DRIVE":
                    self.switch_to_drive_mode()
                
                # SHARE (Button 8) -> Arm Mode
                elif msg.buttons[8] == 1 and self.control_mode != "ARM":
                    self.switch_to_arm_mode()

            # (Optional Xbox Mapping: Start=Drive, Back=Arm)
            elif self.controller_model == "Xbox-360 Controller":
                if msg.buttons[7] == 1 and self.control_mode != "DRIVE":
                    self.switch_to_drive_mode()
                elif msg.buttons[6] == 1 and self.control_mode != "ARM":
                    self.switch_to_arm_mode()

            # -----------------------------------------------------
            # 3. Execute Control Logic Based on Mode
            # -----------------------------------------------------
            if self.control_mode == "DRIVE":
                self.publish_twist_msg(msg)
                self.publish_flipper_msg(msg)
            
            elif self.control_mode == "ARM":
                self.publish_arm_msg(msg)


    def switch_to_arm_mode(self):
        self.control_mode = "ARM"
        self.get_logger().info("Switched to: ARM MODE")
        
        # Safety: Stop Rover driving
        self.cmd_vel_publisher.publish(Twist()) 
        
        # FIX: ALWAYS send the enable signal when Share is pressed, 
        # guaranteeing the Jetson receives it.
        self.arm_enable_pub.publish(Bool(data=True))
        self.arm_enabled = True

        # Update Indicator
        msg = String()
        msg.data = "Blue -> Arm Mode"
        self.indicator_publisher.publish(msg)

    def switch_to_drive_mode(self):
        self.control_mode = "DRIVE"
        self.get_logger().info("Switched to: DRIVE MODE")
        
        # Safety: Stop Arm movement (keeps it holding position)
        zero_vel = Float64MultiArray()
        zero_vel.data = [0.0] * 5
        self.arm_vel_pub.publish(zero_vel)
        
        # Update Indicator
        msg = String()
        msg.data = "Blue -> Drive Mode"
        self.indicator_publisher.publish(msg)

    # --- Mode Switch Helpers ---
    # def switch_to_drive_mode(self):
    #     self.control_mode = "DRIVE"
    #     self.get_logger().info("Switched to: DRIVE MODE")
        
    #     # Safety: Stop Arm
    #     zero_vel = Float64MultiArray()
    #     zero_vel.data = [0.0] * 5
    #     self.arm_vel_pub.publish(zero_vel)
        
    #     # Update Indicator
    #     msg = String()
    #     msg.data = "Blue -> Drive Mode"
    #     self.indicator_publisher.publish(msg)

    # def switch_to_arm_mode(self):
    #     self.control_mode = "ARM"
    #     self.get_logger().info("Switched to: ARM MODE")
        
    #     # Safety: Stop Rover
    #     self.cmd_vel_publisher.publish(Twist()) 
        
    #     # Send Enable Signal to Arm if not already active
    #     if not self.arm_enabled:
    #         self.arm_enable_pub.publish(Bool(data=True))
    #         self.arm_enabled = True


    # --- Drive Functions ---
    def publish_twist_msg(self, joy):
        twist = Twist()
        
        if self.controller_model ==  "Xbox-360 Controller":
            twist.linear.x = (joy.axes[1]/2) + (joy.axes[4]/2)
            twist.angular.z = (joy.axes[0]/2) + (joy.axes[3]/2)
            
        elif self.controller_model ==  "PS4 Controller":
            # Right Stick Up/Down for Forward/Backward
            twist.linear.x = joy.axes[4]
            # Left Stick Left/Right for Turning
            twist.angular.z = joy.axes[0]
            
            # Deadzone
            if -0.17 < twist.linear.x < 0.17: twist.linear.x = 0.0
            if -0.17 < twist.angular.z < 0.17: twist.angular.z = 0.0

        self.cmd_vel_publisher.publish(twist)

    def publish_flipper_msg(self, joy):
        flipper_cmd = Int32MultiArray()
        flipper_cmd.data = [0, 0, 0, 0] 

        if self.controller_model == "PS4 Controller":
            # F1 (Front Left)
            if joy.buttons[4]: flipper_cmd.data[1] = -1   # L1 -> Up
            elif joy.axes[2] < -0.5: flipper_cmd.data[1] = 1 # L2 -> Down
            
            # F2 (Front Right)
            if joy.buttons[5]: flipper_cmd.data[0] = -1   # R1 -> Up
            elif joy.axes[5] < -0.5: flipper_cmd.data[0] = 1 # R2 -> Down

            # F3 (Rear Left) - D-Pad
            if joy.axes[7] > 0.5: flipper_cmd.data[3] = 1
            elif joy.axes[7] < -0.5: flipper_cmd.data[3] = -1

            # F4 (Rear Right) - Triangle/Cross
            if joy.buttons[2]: flipper_cmd.data[2] = 1
            elif joy.buttons[0]: flipper_cmd.data[2] = -1

        self.flipper_publisher.publish(flipper_cmd)

    # --- Arm Functions ---
    def publish_arm_msg(self, joy):
        # 1. Gripper (L1/R1)
        grip_cmd = Int32()
        if self.controller_model == "PS4 Controller":
            if joy.buttons[5] == 1: grip_cmd.data = 1     # R1 Open
            elif joy.buttons[4] == 1: grip_cmd.data = -1  # L1 Close
        self.arm_grip_pub.publish(grip_cmd)

        # 2. Arm Velocities (Joysticks)
        # REVERSED: Added a negative sign to -joy.axes[0] and -joy.axes[3]
        lx = expo(dz(-joy.axes[0], self.ARM_DEADZONE), self.ARM_EXPO) 
        ly = expo(dz(joy.axes[1], self.ARM_DEADZONE), self.ARM_EXPO) 
        rx = expo(dz(-joy.axes[3], self.ARM_DEADZONE), self.ARM_EXPO) 
        ry = expo(dz(joy.axes[4], self.ARM_DEADZONE), self.ARM_EXPO)
        
        yaw_dir = 0.0
        # Check D-Pad axis for Yaw (PS4 D-Pad L/R is usually Axis 6)
        if len(joy.axes) > 7:
            if joy.axes[7] > 0.5: yaw_dir = -1.0    # Left
            elif joy.axes[7] < -0.5: yaw_dir = 1.0 # Right

        vel_msg = Float64MultiArray()
        vel_msg.data = [
            lx * self.ARM_MAX_VEL[0],
            ly * self.ARM_MAX_VEL[1],
            ry * self.ARM_MAX_VEL[2],
            rx * self.ARM_MAX_VEL[3],
            yaw_dir * self.ARM_YAW_SPEED
        ]
        self.arm_vel_pub.publish(vel_msg)

    # --- Utilities ---
    def create_dummy_node(self):
        self.dummyNode = rclpy.create_node('teleop_is_on')
        msg = String()
        msg.data = "Blue -> Manual Mode"
        self.indicator_publisher.publish(msg)
        # Reset to Drive mode when engaging manual
        self.control_mode = "DRIVE" 

    def destroy_dummy_node(self):
        if self.dummyNode is not None:
            self.dummyNode.destroy_node()
            self.dummyNode = None
            msg = String()
            msg.data = "RED -> Autonomous Mode"
            self.indicator_publisher.publish(msg)
            # Send disarm signal to arm for safety
            self.arm_enable_pub.publish(Bool(data=False))
            self.arm_enabled = False

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
