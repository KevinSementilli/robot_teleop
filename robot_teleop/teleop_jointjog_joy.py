import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from control_msgs.msg import JointJog

class TeleopJointJogJoy(Node):

    def __init__(self):
        super().__init__("teleop_jointjog_joy")

        # Declare parameters that can be set via launch file or ros2 parameter system
        self.declare_parameter("joint_names", ["base", "arm1", "arm2", "arm3", "claw"])
        self.declare_parameter("joint_speed", 0.8)
        self.declare_parameter("claw_speed", 0.6)

        # Retrieve parameters
        self.joint_names = self.get_parameter("joint_names").get_parameter_value().string_array_value
        self.joint_speed = self.get_parameter("joint_speed").get_parameter_value().double_value
        self.claw_speed = self.get_parameter("claw_speed").get_parameter_value().double_value

        # Publisher to send joint velocity commands
        self.publisher = self.create_publisher(JointJog, '/teleop_vel', 10)
        
        # Subscription to /joy topic to receive joystick input
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        # Initialize joint velocities based on the length of joint names
        self.velocities = [0.0] * len(self.joint_names)

    def joy_callback(self, msg):
        """
        Process joystick input and update joint velocities accordingly.
        """
        # Control the base, arm1, arm2, and arm3 (or more joints dynamically)
        for i in range(4):  # You can update this loop if there are more joints
            self.velocities[i] = msg.axes[i] * self.joint_speed

        # Call the helper function for claw control logic
        self.control_claws(msg)

        # Create and publish the JointJog message
        joint_jog_msg = JointJog()
        joint_jog_msg.joint_names = self.joint_names
        joint_jog_msg.velocities = self.velocities
        joint_jog_msg.duration = 0.0  # Continuous motion
        self.publisher.publish(joint_jog_msg)

    def control_claws(self, msg):
        """
        Helper function to control claw(s) based on joystick input.
        """
        # Handle claws based on the number of joints for claws
        if len(self.joint_names) > 4:  # Multiple claw joints (simulated or other configurations)
            if len(self.joint_names) == 6:  # Two claw joints
                if msg.axes[4] == -1.0:  # Right Trigger (RT)
                    self.velocities[4] = self.claw_speed    # Left claw forward
                    self.velocities[5] = -self.claw_speed   # Right claw backward
                elif msg.axes[5] == -1.0:  # Left Trigger (LT)
                    self.velocities[4] = -self.claw_speed   # Left claw backward
                    self.velocities[5] = self.claw_speed    # Right claw forward
                else:
                    self.velocities[4] = 0.0
                    self.velocities[5] = 0.0
            else:  # More than 1 claw joint (if any other configuration)
                # Add logic for handling more than 2 claw joints if necessary
                pass
        else:  # Single claw joint (real hardware or simple configuration)
            if msg.axes[4] == -1.0:  # Right Trigger (RT)
                self.velocities[4] = self.claw_speed    # Claw forward
            elif msg.axes[5] == -1.0:  # Left Trigger (LT)
                self.velocities[4] = -self.claw_speed   # Claw backward
            else:
                self.velocities[4] = 0.0

def main(args=None):
    rclpy.init(args=args)
    teleop_jointjog_joy = TeleopJointJogJoy()
    rclpy.spin(teleop_jointjog_joy)

    teleop_jointjog_joy.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
