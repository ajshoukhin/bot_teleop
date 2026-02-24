#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32, Bool, String
from sensor_msgs.msg import Joy

# --- Helper Functions ---
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


class ArmTeleopNode(Node):
    def __init__(self):
        super().__init__('arm_teleop_node')

        # --- Settings ---
        self.DEADZONE = 0.10
        self.EXPO = 1.6
        self.MAX_VEL = [0.8, 0.8, 0.8, 1.2, 1.2]
        self.YAW_SPEED = 0.8

        # --- Internal State ---
        self.is_armed = False
        self.controller_model = "PS4 Controller" # Default fallback

        # --- Subscribers ---
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.status_sub = self.create_subscription(String, '/arm/status', self.status_callback, 10)

        # --- Publishers ---
        self.vel_pub = self.create_publisher(Float64MultiArray, '/arm/cmd_vel', 10)
        self.grip_pub = self.create_publisher(Int32, '/arm/cmd_grip', 10)
        self.enable_pub = self.create_publisher(Bool, '/arm/enable', 10)

        self.get_logger().info("Dedicated Arm Teleop Node Started.")
        self.get_logger().info("Controls: Cross=ARM, Circle=HOLD/DISARM, L1/R1=Gripper, Joysticks=Move")

        # --- AUTO-ARM ON STARTUP ---
        # 0.5 sec delay ensures the ROS 2 network has time to map topics 
        # before we blast the enable message.
        self.startup_timer = self.create_timer(0.5, self.auto_arm_callback)

    def auto_arm_callback(self):
        if not self.is_armed:
            self.is_armed = True
            self.enable_pub.publish(Bool(data=True))
            self.get_logger().info("AUTO-ARMED: Sent enable signal to Jetson on startup.")
        
        # Cancel the timer so it only runs once
        self.startup_timer.cancel()

    def status_callback(self, msg):
        # Print the Jetson's hardware state to the laptop terminal
        # throttle_duration_sec=1.0 prevents the terminal from being spammed
        self.get_logger().info(msg.data, throttle_duration_sec=1.0)

    def detect_controller_model(self, msg):
        if len(msg.buttons) == 11 and len(msg.axes) == 8:
            self.controller_model = "Xbox-360 Controller"
        elif len(msg.buttons) == 13 and len(msg.axes) == 8:
            self.controller_model = "PS4 Controller"

    def joy_callback(self, msg):
        self.detect_controller_model(msg)

        # -----------------------------------------------------
        # 1. Arming / Disarming (Cross / Circle)
        # -----------------------------------------------------
        if self.controller_model == "PS4 Controller":
            btn_arm = msg.buttons[0]    # Cross
            btn_hold = msg.buttons[1]   # Circle
            btn_l1 = msg.buttons[4]
            btn_r1 = msg.buttons[5]
        else: # Xbox mappings fallback
            btn_arm = msg.buttons[0]    # A
            btn_hold = msg.buttons[1]   # B
            btn_l1 = msg.buttons[4]     # LB
            btn_r1 = msg.buttons[5]     # RB

        # Manual Arm/Disarm Override
        if btn_arm == 1 and not self.is_armed:
            self.is_armed = True
            self.enable_pub.publish(Bool(data=True))
            self.get_logger().info("MANUAL ARM: Sending enable signal.")
            
        elif btn_hold == 1 and self.is_armed:
            self.is_armed = False
            self.enable_pub.publish(Bool(data=False))
            self.get_logger().info("HOLD: Sending disarm signal.")

        # -----------------------------------------------------
        # 2. Gripper Control (L1 / R1)
        # -----------------------------------------------------
        grip_cmd = Int32()
        if btn_r1 == 1 and btn_l1 == 0:
            grip_cmd.data = 1     # Open
        elif btn_l1 == 1 and btn_r1 == 0:
            grip_cmd.data = -1    # Close
        else:
            grip_cmd.data = 0     # Hold
            
        self.grip_pub.publish(grip_cmd)

        # -----------------------------------------------------
        # 3. Arm Velocities (Joysticks)
        # -----------------------------------------------------
        # If not armed, don't bother calculating or publishing velocities
        if not self.is_armed:
            return

        # ROS Joy standardizes axes to [-1.0, 1.0], Up/Left is positive.
        lx_raw = msg.axes[0]
        ly_raw = msg.axes[1] 
        rx_raw = msg.axes[3]
        ry_raw = msg.axes[4]

        # Apply deadzone and expo curve
        lx = expo(dz(lx_raw, self.DEADZONE), self.EXPO)
        ly = expo(dz(ly_raw, self.DEADZONE), self.EXPO)
        rx = expo(dz(rx_raw, self.DEADZONE), self.EXPO)
        ry = expo(dz(ry_raw, self.DEADZONE), self.EXPO)
        
        # D-pad for Yaw (Usually Axis 6 on PS4 in Linux)
        yaw_dir = 0.0
        if len(msg.axes) > 6:
            if msg.axes[6] > 0.5:
                yaw_dir = 1.0    # Left
            elif msg.axes[6] < -0.5:
                yaw_dir = -1.0   # Right

        # Map to Joints [J0, J1, J2, J3, J4]
        vel_msg = Float64MultiArray()
        vel_msg.data = [
            lx * self.MAX_VEL[0],
            ly * self.MAX_VEL[1],
            ry * self.MAX_VEL[2], 
            rx * self.MAX_VEL[3],
            yaw_dir * self.YAW_SPEED
        ]
        
        self.vel_pub.publish(vel_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArmTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Safety disarm on shutdown
        node.enable_pub.publish(Bool(data=False))
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
