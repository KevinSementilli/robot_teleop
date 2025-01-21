import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray

class TeleopVel(Node):

    def __init__(self):
        super().__init__("teleop_vel")

        # Declare DEFAULT parameters 
        self.declare_parameter("joint_names", ["base", "arm1", "arm2", "arm3", "claw"])
        self.declare_parameter("joint_speed", 0.8)
        self.declare_parameter("claw_speed", 0.6)

        # Retrieve config parameters
        self.joint_names = self.get_parameter("joint_names").get_parameter_value().string_array_value
        self.joint_speed = self.get_parameter("joint_speed").get_parameter_value().double_value
        self.claw_speed = self.get_parameter("claw_speed").get_parameter_value().double_value

        # Publisher to send joint velocity commands
        self.publisher = self.create_publisher(Float64MultiArray, '/vel_controller/commands', 10)
        
        # Subscription to /joy topic to receive joystick input
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        # Initialize joint velocities based on the length of joint names
        self.velocities = [0.0] * len(self.joint_names)

        # Flag to print the log only once
        self.has_logged = False

    def joy_callback(self, msg):
        """
        Process joystick input and update joint velocities accordingly.
        """
        # Control the base, arm1, arm2, and arm3 (or more joints dynamically)
        for i in range(4):
            self.velocities[i] = msg.axes[i] * self.joint_speed

        # Call the helper function for claw control logic
        self.control_claws(msg)

        # Create and publish the Float64MultiArray message
        float_array_msg = Float64MultiArray()
        float_array_msg.data = self.velocities
        self.publisher.publish(float_array_msg)

        # Log an info message
        if not self.has_logged:
            joint_names_str = ', '.join(self.joint_names)  # Convert joint names to a comma-separated string
            self.get_logger().info(f"Publishing joint velocities: {joint_names_str}")
            self.has_logged = True

    def control_claws(self, msg):
        """
        Function to control claw(s) based on joystick input.
        """
        # Handle claws based on the number of joints for claws
        if len(self.joint_names) > 5:  # Multiple claw joints (simulated or other configurations)
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
    teleop_vel = TeleopVel()
    rclpy.spin(teleop_vel)

    teleop_vel.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
