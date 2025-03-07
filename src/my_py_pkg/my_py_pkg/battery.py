#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv._set_led import SetLed

class BatteryNode(Node):
  def __init__(self):
    super().__init__("battery")
    self.battery_state_ = "full"
    self.last_timer_battery_state_changes = self.get_current_time_seconds()
    self.battery_timer_ = self.create_timer(0.1, self.check_battery_state)
    self.set_led_client_ = self.create_client(SetLed, "set_led")
    self.get_logger().info("Battery node has been started")
    
  def get_current_time_seconds(self):
    seconds, nanoseconds = self.get_clock().now().seconds_nanoseconds()
    return seconds + nanoseconds/1000000000.0
  
  def check_battery_state(self):
    time_now = self.get_current_time_seconds()
    if self.battery_state_ == "full":
      if time_now - self.last_timer_battery_state_changes > 4.0:
        self.battery_state_ = "empty"
        self.get_logger().info("Battery is empty! Charging...")
        self.last_timer_battery_state_changes = time_now
        self.call_set_led(2, 1)
    elif self.battery_state_ == "empty":
      if time_now - self.last_timer_battery_state_changes > 6.0:
        self.battery_state_ = "full" 
        self.get_logger().info("Battery is full!")
        self.last_timer_battery_state_changes = time_now
        self.call_set_led(2, 0)

  def call_set_led(self, led_number, state):
    while not self.set_led_client_.wait_for_service(1.0):
      self.get_logger().warn("Waiting for Set Led Service")

    req = SetLed.Request()
    req.led_number = led_number
    req.state = state
    
    future = self.set_led_client_.call_async(req)
    future.add_done_callback(self.callback_call_set_led)

  def callback_call_set_led(self, future):
    rsp = future.result()
    self.get_logger().info("Response: " + str(rsp.success))

def main(args=None):
  rclpy.init(args=args)
  node = BatteryNode()
  rclpy.spin(node)
  rclpy.shutdown()
  
if __name__ == "__main__":
  main()