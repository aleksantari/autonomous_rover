import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class Auton(Node):

    def __init__(self):
        super().__init__('auton')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.declare_parameter("linear_speed_limit",1.0)
        self.declare_parameter("angular_speed_limit",1.0)
        self.linenar_speed_limit = self.get_parameter("linear_speed_limit").get_parameter_value().double_value
        self.angular_speed_limit = self.get_parameter("angular_speed_limit").get_parameter_value().double_value

    def startup(self):
        rclpy.init()
        self.get_logger().info("Autonomous control node has started.")
        self.go()
        self.get_logger().info("Autonomous control node has been stopped.")
        rclpy.shutdown()
    
    def forward(self):
        twist = Twist()
        twist.linear.x = 0.3 * self.linenar_speed_limit
        twist.angular.z = 0.0
        self.pub.publish(twist)
        time.sleep(2)
        twist.linear.x = 0.0
        self.pub.publish(twist)
    def left(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.3 * self.angular_speed_limit
        self.pub.publish(twist)
        time.sleep(2)
        twist.linear.x = 0.0
        self.pub.publish(twist)
    def right(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = -0.3 * self.angular_speed_limit
        self.pub.publish(twist)
        time.sleep(2)
        twist.linear.x = 0.0
        self.pub.publish(twist)
    def go(self):
        self.forward()
        self.left()
        self.forward()
        self.right()
    
a = Auton()
a.startup()