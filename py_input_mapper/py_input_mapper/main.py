import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import math

# Global variables
publisher = None
node = None

def joy_callback(msg):
    global publisher
    
    # Assuming the right joystick axes are axes 3 and 4
    right_joystick_x = msg.axes[3]  # X axis
    right_joystick_y = msg.axes[4]  # Y axis
    
    # Linear velocity and angular velocity limits
    max_linear_velocity = 1.0
    max_angular_velocity = 1.0
    
    # Compute linear and angular velocities
    linear_velocity = max_linear_velocity * right_joystick_y
    angular_velocity = max_angular_velocity * right_joystick_x
    
    # Create a Twist message
    twist_msg = Twist()
    twist_msg.linear.x = linear_velocity
    twist_msg.angular.z = angular_velocity
    
    # Publish the message
    publisher.publish(twist_msg)
    node.get_logger().info(
        f'Publishing: Linear velocity: {twist_msg.linear.x}, Angular velocity: {twist_msg.angular.z}'
    )

def main(args=None):
    global publisher, node
    
    rclpy.init(args=args)
    
    # Create a node
    node = Node('joystick_to_twist')
    
    # Create a publisher for the cmd_velocity topic
    publisher = node.create_publisher(Twist, 'cmd_velocity', 10)
    
    # Create a subscription to the joy topic
    subscription = node.create_subscription(
        Joy,
        'joy',
        joy_callback,
        10
    )
    
    # Spin to keep the node running
    rclpy.spin(node)
    
    # Cleanup
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
