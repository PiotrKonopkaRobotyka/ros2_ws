#! /usr/bin/env python3

"""
Description:
    This Ros 2 node periodically publishes "Hello World" messages to a topic.
    
-------    
Publishing Topics
    The channel notaining the "Hello worsld" messages
    /py_example_topic - std_msgs/String

Subscription Topics:
    None
 --------
Author: Piotr Konopka
Date June 03, 2025
    """
import rclpy #import the ROS 2 client library for Python
from rclpy.node import Node # import the node class, used for creating nodes

from std_msgs.msg import String #import String message type for ROS 2


class MinimalPyPublisher(Node):
    """Create a minimal publisher node.
    """

    def __init__(self):
        """Create a custom node class for publishing messages
        """
        #Initialize the node witha name

        super().__init__('minimal_py_publisher')

        #create a publisher on the topic with a queue size of 10 messages
        self.publisher_1 = self.create_publisher(String, '/py_example_topic', 10)

        #creata a timer witha a period of 0.5 seconds to trigger publishinf of message
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

        #Initialize a counter veriable for message content
        self.i = 0 
    
    def timer_callback(self):
        """Callback function executed periodically by the timer"""
        #Create a new String message object
        msg = String ()

        #Set the message content
        msg.data = 'Hello World: %d' %self.i

        #Publish the message you created aboce to the topic
        self.publisher_1.publish(msg)

        #log a message indicating the message has been published
        self.get_logger().info('Publishing: "%s"' %msg.data)

        self.i =self.i +1

def main (args=None):
    """Main function to start the ROS2 node
    Args:
        args(list,optional): command-line arguments. Default to none
    """

    rclpy.init(args=args)

    #Create and instance of the Minimal Publisher node
    minimal_py_publisher = MinimalPyPublisher()
    
    rclpy.spin(minimal_py_publisher)

    #Destroy the node expliicity
    minimal_py_publisher.destroy_node()

    #Shutdown the ROS 2 comunication
    rclpy.shutdown()

if __name__ == '__main__':
    #Execute the main function if the script is run directly
    main()