#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg._led_state_array import LedStateArray
from my_robot_interfaces.srv._set_led import SetLed

class LedPanelNode(Node):
  def __init__(self):
    super().__init__("led_panel")
    self.declare_parameter("led_states", [0,0,0])
    self.led_states_ = self.get_parameter("led_states").value
    self.led_state_pub_ = self.create_publisher(LedStateArray, "led_sts", 10)
    self.led_state_timer_ = self.create_timer(5.0, self.publish_led_states)
    
    self.led_state_server_ = self.create_service(SetLed, "set_led", self.callback_set_led)
    
    self.get_logger().info("LED Panel Node has been online")
  
  def publish_led_states(self):
    msg = LedStateArray()
    msg.led_states = self.led_states_
    
    self.led_state_pub_.publish(msg)
    
  def callback_set_led(self, request: SetLed.Request, response: SetLed.Response):
    led_number = request.led_number
    state = request.state
    
    if led_number >= len(self.led_states_) or led_number < 0 :
      response.success = False
    elif state not in [0, 1]:
      response.success = False
    else:
      self.led_states_[led_number] = state
      self.publish_led_states()
      response.success = True
    
    return response
    
  
def main(args=None):
  rclpy.init(args=args)
  node = LedPanelNode()
  rclpy.spin(node)
  rclpy.shutdown()
  
if __name__ == "__main__":
  main()