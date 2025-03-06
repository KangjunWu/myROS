#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import HardwareStatus

class HardwareStatusPublisherNode(Node):
  def __init__(self):
    super().__init__("hardware_status_publisher")
    self.hw_status_pub = self.create_publisher(HardwareStatus, "hw_sts", 10)
    self.create_timer(timer_period_sec=1.0, callback=self.timer_callback)
    self.get_logger().info("HW status publisher has been started")
    
  def timer_callback(self):
    msg = HardwareStatus()
    msg.temperature = 43.6
    msg.are_motors_ready = True
    msg.debug_message = "Nothing Special"
    self.hw_status_pub.publish(msg)

def main(args=None):
  rclpy.init(args=args)
  node = HardwareStatusPublisherNode()
  rclpy.spin(node)
  rclpy.shutdown()
  
if __name__ == "__main__":
  main()