import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Global variables
left_wheel_speed = 0.0
right_wheel_speed = 0.0

def clamp(value, min_value, max_value):
    """
    Clamp the value between min_value and max_value.

    Parameters:
    value (float): The value to be clamped.
    min_value (float): The minimum allowed value.
    max_value (float): The maximum allowed value.

    Returns:
    float: The clamped value.
    """
    return max(min_value, min(value, max_value))

def scale_speed(speed):
    # Clamp speed to the range of -1 to 1
    clamped_speed = clamp(speed, -1, 1)
    
    # Scale normalized speed to the range of -100 to 100
    return int(clamped_speed * 100)

def cmd_velocity_callback(msg):
    global left_wheel_speed, right_wheel_speed
    
    # Linear and angular velocities
    linear_velocity = msg.linear.x
    angular_velocity = msg.angular.z
    
    # Wheelbase (distance between left and right wheels)
    wheelbase = 0.5  # Example value in meters
    
    # Compute left and right wheel speeds
    left_wheel_speed = linear_velocity - (angular_velocity * wheelbase / 2.0)
    right_wheel_speed = linear_velocity + (angular_velocity * wheelbase / 2.0)
    
    # Scale the wheel speeds to -100 to 100
    scaled_left_speed = scale_speed(left_wheel_speed)
    scaled_right_speed = scale_speed(right_wheel_speed)
    
    # Control the wheels based on computed speeds
    drive_wheels(scaled_left_speed, scaled_right_speed)

def drive_wheels(left_speed, right_speed):
    # This function should send the wheel speed commands to the robot's hardware
    # Replace this print statement with actual wheel control code
    print(f"Left wheel speed: {left_speed}, Right wheel speed: {right_speed}")
    if left_speed < 0:
       left_speed = abs(left_speed)
       print(f"Left Backward at {left_speed}")
    else:
       print(f"Left Forward at {left_speed}")

    if right_speed < 0:
       right_speed = abs(right_speed)
       print(f"Right Backward at {right_speed}")
    else:
       print(f"Right Forward at {right_speed}")




def main(args=None):
    global node
    
    rclpy.init(args=args)
    
    # Create a node
    node = Node('wheel_driver')
    
    # Create a subscription to the cmd_velocity topic
    subscription = node.create_subscription(
        Twist,
        'cmd_velocity',
        cmd_velocity_callback,
        10
    )
    
    # Spin to keep the node running
    rclpy.spin(node)
    
    # Cleanup
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
