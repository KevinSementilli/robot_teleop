import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from control_msgs.msg import JointJog


class TeleopVel(Node):

    def __init__(self):
        super().__init__("teleop_vel")

        # Publisher to send joint velocity commands
        self.publisher = self.create_publisher(JointJog, '/teleop_vel', 10)
        
        # Subscription to /joy topic to receive joystick input
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
         
        # Define the joint names and initialize velocities
        self.joint_names = ['base', 'arm1', 'arm2', 'arm3', 'claw']
        self.velocities = [0.0] * len(self.joint_names)
    
    def joy_callback(self, msg):
        # Define speeds for joints and claw
        joint_speed = 0.8  # rad/sec
        claw_speed = 0.6  # rad/sec

        # Control the base, arm1, arm2, and arm3 with the left and right sticks
        self.velocities[0] = msg.axes[0] * joint_speed  # Left stick X
        self.velocities[1] = msg.axes[1] * joint_speed  # Left stick Y
        self.velocities[2] = msg.axes[2] * joint_speed  # Right stick X
        self.velocities[3] = msg.axes[3] * joint_speed  # Right stick Y
        
        # Control the claw using the triggers (RT and LT)
        if msg.axes[4] == -1.0:  # Right Trigger (RT)
            self.velocities[4] = claw_speed  # Move the claw forward
        elif msg.axes[5] == -1.0:  # Left Trigger (LT)
            self.velocities[4] = -claw_speed  # Move the claw backward
        else:
            self.velocities[4] = 0.0  # No claw movement when no trigger is pressed

        # Create and publish the JointJog message
        joint_jog_msg = JointJog()
        joint_jog_msg.joint_names = self.joint_names
        joint_jog_msg.velocities = self.velocities
        joint_jog_msg.duration = 0.0  # Continuous motion
        self.publisher.publish(joint_jog_msg)  # Publish the message


def main(args=None):
    rclpy.init(args=args)
    teleop_vel = TeleopVel()
    rclpy.spin(teleop_vel)

    teleop_vel.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
