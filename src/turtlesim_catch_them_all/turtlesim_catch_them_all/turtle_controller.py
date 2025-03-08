#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from my_robot_interfaces.msg import Turtle
from my_robot_interfaces.msg import TurtleArray
from my_robot_interfaces.srv import CatchTurtle
from functools import partial

class TurtleControllerNode(Node):
  def __init__(self):
    super().__init__("turtle_controller")
    self.turtle_to_catch: Turtle = None
    self.declare_parameter("catch_closest_turtle_first", True)
    self.catch_closest_turtle_first_ = self.get_parameter("catch_closest_turtle_first").value
    self.pose_ : Pose = None
    self.pose_subscriber = self.create_subscription(Pose, "/turtle1/pose", self.callback_pose, 10)
    self.cmd_vel_publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
    self.controller_loop_timer_ = self.create_timer(0.01, self.control_loop)
    self.alive_turtles_subscriber_ = self.create_subscription(TurtleArray, "alive_turtles", self.callback_alive_turtles, 10)
    self.catch_turtle_client_ = self.create_client(CatchTurtle, "catch_turtle")
      
    self.get_logger().info("Turtle Controller Node has been started")
    
  def call_catch_turtle_service(self, turtle_name):
    while not self.catch_turtle_client_.wait_for_service(1.0):
      self.get_logger().warn("Waiting for catch turtle service.../")
      
    request = CatchTurtle.Request()
    request.name = turtle_name
    future = self.catch_turtle_client_.call_async(request)
    future.add_done_callback(partial(self.callback_call_catch_turtle_service, request=request))
      
  def callback_call_catch_turtle_service(self, future, request: CatchTurtle.Request):
    response :CatchTurtle.Response = future.result()
    if not response.success :
      self.get_logger().error("Turtle " + request.name + " could not be removed")
    
  def callback_alive_turtles(self, msg: TurtleArray):
    if len(msg.tutles) > 0:
      if self.catch_closest_turtle_first_:
        closest_turtle = None
        closest_turtle_distance = None
        for turtle in msg.tutles:
          dist_x = turtle.x - self.pose_.x
          dist_y = turtle.y - self.pose_.y
          dist = math.sqrt(dist_x*dist_x + dist_y*dist_y)
          if closest_turtle is None or dist < closest_turtle_distance:
            closest_turtle =turtle
            closest_turtle_distance = dist
        if closest_turtle is not None:
          self.turtle_to_catch = closest_turtle
      else:
        self.turtle_to_catch = msg.tutles[0]

  def callback_pose(self, pose:Pose):
    self.pose_ = pose
    
  def control_loop(self):
    if self.pose_ == None or self.turtle_to_catch == None:
      return
    
    dist_x = self.turtle_to_catch.x- self.pose_.x
    dist_y = self.turtle_to_catch.y - self.pose_.y
    dist = math.sqrt(dist_x * dist_x + dist_y * dist_y)
    
    cmd = Twist()
    
    if dist > 0.5:
      # position
      cmd.linear.x = 2*dist
      
      # orientation
      goal_theta = math.atan2(dist_y, dist_x)
      diff = goal_theta - self.pose_.theta
      if diff > math.pi:
        diff -= 2*math.pi
      elif diff < -math.pi:
        diff += 2*math.pi
      
      cmd.angular.z = 6*diff
    else:
      # target reached
      cmd.linear.x = 0.0
      cmd.angular.z = 0.0
      self.call_catch_turtle_service(self.turtle_to_catch.name)
      self.turtle_to_catch = None
    
    self.cmd_vel_publisher_.publish(cmd)
    

def main(args=None):
  rclpy.init(args=args)
  node = TurtleControllerNode()
  rclpy.spin(node)
  rclpy.shutdown()
  
if __name__ == "__main__":
  main()