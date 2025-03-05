#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class RobotNewStationNode(Node):
  def __init__(self):
    super().__init__("robot_news_station")
    self.publisher_ = self.create_publisher(String, "robot_news", 10)
    self.timer_ = self.create_timer(1, self.timerCallback)
    self.get_logger().info("Robot News Station has been started")
    
  def publish_news(self):
    msg = String()
    msg.data = "Hello"
    self.publisher_.publish(msg)
    
  def timerCallback(self):
    self.publish_news()
  
def main(args=None):
  rclpy.init(args=args)
  node = RobotNewStationNode()
  rclpy.spin(node)
  rclpy.shutdown()
  
if __name__ == "__main__":
  main()