#!/usr/bin/env python3
import rclpy # Import ROS2 python interface
def main(args=None):
 rclpy.init(args=args) # Initialize the ROS2 python interface
 node = rclpy.create_node('python_hello_world') # Node constructor, give it a name
 node.get_logger().info("Hello, ROS2!") # Use the ROS2 node's built in logger
 node.destroy_node() # Node destructor
 rclpy.shutdown() # Shut down the ROS2 python interface
# Check if the script is being run directly
if __name__ == '__main__':
 main()