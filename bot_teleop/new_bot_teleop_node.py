#!/usr/bin/env python3
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
        self.arm_enabled = False       # Tracks if the MIT motors are energized

        # Flipper Sync State
        self.sync_flipper_mode = False
        self.last_ps_btn = 0

        # Arm Config
        self.ARM_DEADZONE = 0.10
        self.ARM_EXPO = 1.6
        self.ARM_MAX_VEL = [0.3, 0.6, 0.3, 0.4, 0.4]
        self.ARM_YAW_SPEED = 0.4 #increased from 0.2 to 0.4

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
            # This also acts as an Emergency Disarm for the Arm!
            elif msg.buttons[1] == 1 and self.dummyNode is not None:
                self.send_msg = False
                self.destroy_dummy_node()
                self.get_logger().info("AUTONOMOUS MODE ENGAGED")

        elif self.controller_model == "Xbox-360 Controller":
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
                    
                # PS Button (10) -> Toggle Flipper Sync Mode (Only in Drive Mode)
                current_ps = msg.buttons[10]
                if current_ps == 1 and self.last_ps_btn == 0 and self.control_mode == "DRIVE":
                    self.sync_flipper_mode = not self.sync_flipper_mode
                    mode_str = "SYNCED (Pairs)" if self.sync_flipper_mode else "INDEPENDENT"
                    self.get_logger().info(f"Flipper Mode Switched To: {mode_str}")
                self.last_ps_btn = current_ps

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
        
        # Safety: Stop Rover driving
        self.cmd_vel_publisher.publish(Twist()) 
        
        # --- NEW: Auto-Arm on Mode Switch ---
        if not self.arm_enabled:
            self.arm_enabled = True
            self.arm_enable_pub.publish(Bool(data=True))
            self.get_logger().info("Switched to: ARM MODE (Auto-Energized).")
        
        # Update Indicator
        msg = String()
        msg.data = "Blue -> Arm Mode (ARMED)"
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

    # --- Drive Functions ---
    def publish_twist_msg(self, joy):
        twist = Twist()
        if self.controller_model ==  "Xbox-360 Controller":
            twist.linear.x = (joy.axes[1]/2) + (joy.axes[4]/2)
            twist.angular.z = (joy.axes[0]/2) + (joy.axes[3]/2)
        elif self.controller_model ==  "PS4 Controller":
            twist.linear.x = joy.axes[4]
            twist.angular.z = joy.axes[0]
            if -0.17 < twist.linear.x < 0.17: twist.linear.x = 0.0
            if -0.17 < twist.angular.z < 0.17: twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)

    def publish_flipper_msg(self, joy):
        flipper_cmd = Int32MultiArray()
        flipper_cmd.data = [0, 0, 0, 0] 

        if self.controller_model == "PS4 Controller":
            if self.sync_flipper_mode:
                # SYNCED MODE (Pairs)
                if joy.buttons[4] == 1:       # L1 -> Both UP
                    flipper_cmd.data[1] = -1
                    flipper_cmd.data[0] = 1   
                elif joy.axes[2] < -0.5:      # L2 -> Both DOWN
                    flipper_cmd.data[1] = 1   
                    flipper_cmd.data[0] = -1  

                if joy.axes[7] > 0.5:         # D-Pad UP -> Both UP
                    flipper_cmd.data[3] = 1   
                    flipper_cmd.data[2] = -1  
                elif joy.axes[7] < -0.5:      # D-Pad DOWN -> Both DOWN
                    flipper_cmd.data[3] = -1  
                    flipper_cmd.data[2] = 1   
            else:
                # INDEPENDENT MODE (Original)
                if joy.buttons[4]: flipper_cmd.data[1] = -1   
                elif joy.axes[2] < -0.5: flipper_cmd.data[1] = 1 
                if joy.buttons[5]: flipper_cmd.data[0] = 1   
                elif joy.axes[5] < -0.5: flipper_cmd.data[0] = -1 
                if joy.axes[7] > 0.5: flipper_cmd.data[3] = 1
                elif joy.axes[7] < -0.5: flipper_cmd.data[3] = -1
                if joy.buttons[2]: flipper_cmd.data[2] = -1
                elif joy.buttons[0]: flipper_cmd.data[2] = 1
        self.flipper_publisher.publish(flipper_cmd)

    # --- Arm Functions ---
    def publish_arm_msg(self, joy):
        # 1. Explicit Arming Logic inside Arm Mode
        if self.controller_model == "PS4 Controller":
            btn_arm = joy.buttons[0]      # Cross
            btn_disarm = joy.buttons[2]   # Triangle
            btn_r1 = joy.buttons[5]
            btn_l1 = joy.buttons[4]
        else:
            btn_arm = joy.buttons[0]      # A
            btn_disarm = joy.buttons[3]   # Y
            btn_r1 = joy.buttons[5]
            btn_l1 = joy.buttons[4]

        if btn_arm == 1 and not self.arm_enabled:
            self.arm_enabled = True
            self.arm_enable_pub.publish(Bool(data=True))
            self.get_logger().info("ARM ENERGIZED.")
            msg = String()
            msg.data = "Blue -> Arm Mode (ARMED)"
            self.indicator_publisher.publish(msg)
            
        elif btn_disarm == 1 and self.arm_enabled:
            self.arm_enabled = False
            self.arm_enable_pub.publish(Bool(data=False))
            self.get_logger().info("ARM DISARMED (Limp).")
            msg = String()
            msg.data = "Blue -> Arm Mode (IDLE)"
            self.indicator_publisher.publish(msg)

        # 2. Gripper (L1/R1)
        grip_cmd = Int32()
        if btn_r1 == 1: grip_cmd.data = 1     # Open
        elif btn_l1 == 1: grip_cmd.data = -1  # Close
        self.arm_grip_pub.publish(grip_cmd)

        # 3. Arm Velocities (Joysticks) - Only calculate if armed
        if not self.arm_enabled:
            return

        lx = expo(dz(-joy.axes[0], self.ARM_DEADZONE), self.ARM_EXPO) 
        ly = expo(dz(joy.axes[1], self.ARM_DEADZONE), self.ARM_EXPO) 
        rx = expo(dz(-joy.axes[3], self.ARM_DEADZONE), self.ARM_EXPO) 
        ry = expo(dz(-joy.axes[4], self.ARM_DEADZONE), self.ARM_EXPO) #md 3/7/26
        
        yaw_dir = 0.0
        # Check explicitly for Axis 6 to prevent silent stall burnout
        if len(joy.axes) > 6:
            if joy.axes[6] > 0.5: yaw_dir = -1.0    # Left
            elif joy.axes[6] < -0.5: yaw_dir = 1.0  # Right

        vel_msg = Float64MultiArray()
        vel_msg.data = [
            lx * self.ARM_MAX_VEL[0],
            ly * self.ARM_MAX_VEL[1],
            ry * self.ARM_MAX_VEL[3], #2
            rx * self.ARM_MAX_VEL[2], #3
            yaw_dir * self.ARM_YAW_SPEED
        ]
        self.arm_vel_pub.publish(vel_msg)

    # --- Utilities ---
    def create_dummy_node(self):
        self.dummyNode = rclpy.create_node('teleop_is_on')
        msg = String()
        msg.data = "Blue -> Manual Mode"
        self.indicator_publisher.publish(msg)
        self.control_mode = "DRIVE" 

    def destroy_dummy_node(self):
        if self.dummyNode is not None:
            self.dummyNode.destroy_node()
            self.dummyNode = None
            msg = String()
            msg.data = "RED -> Autonomous Mode"
            self.indicator_publisher.publish(msg)
            # HARD SAFETY: Disable arm when dropping out of manual mode
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
