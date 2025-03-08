#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from functools import partial
import random
import math
from my_robot_interfaces.msg import Turtle
from my_robot_interfaces.msg import TurtleArray
from my_robot_interfaces.srv import CatchTurtle
from turtlesim.srv import Kill
from functools import partial

class TurtleSpawnerNode(Node):
  def __init__(self):
    super().__init__("turtle_spawner")
    self.declare_parameter("turtle_name_prefix", "turtle")
    self.turtle_name_perfix_ = self.get_parameter("turtle_name_prefix").value
    self.declare_parameter("spawn_frequency", 1.0)
    self.spawn_timer_period_ = 1.0 / self.get_parameter("spawn_frequency").value
    
    self.turtle_counter_ = 0
    self.alive_turtles_ = []
    self.alive_turtles_publisher_ = self.create_publisher(TurtleArray, "alive_turtles", 10)
    self.spawn_client_ = self.create_client(Spawn, "/spawn")
    self.spawn_timer_ = self.create_timer(self.spawn_timer_period_, self.spawn_new_turtle)
    self.catch_turtle_service_ = self.create_service(CatchTurtle, "catch_turtle", self.callback_catch_turtle)
    self.kill_turtle_client_ = self.create_client(Kill, "/kill")

    self.get_logger().info("Turtle Spawner Node has been started")
    
  def call_kill_service(self, turtle_name):
    while not self.kill_turtle_client_.wait_for_service(1.0):
      self.get_logger().warn("Waiting for kill service.../")
      
    request = Kill.Request()
    request.name = turtle_name
    future = self.kill_turtle_client_.call_async(request)
    future.add_done_callback(partial(self.callback_call_kill_service, request=request))
      
  def callback_call_kill_service(self, future, request: Kill.Request):
    for (i, turtle) in enumerate(self.alive_turtles_):
      if turtle.name == request.name:
        del self.alive_turtles_[i]
        self.publish_alive_turtiles()
        break
    
  def callback_catch_turtle(self, request: CatchTurtle.Request, response: CatchTurtle.Response):
    self.call_kill_service(request.name)
    response.success = True
    return response
  
  def publish_alive_turtiles(self):
    msg = TurtleArray()
    msg.tutles = self.alive_turtles_
    self.alive_turtles_publisher_.publish(msg)
    
  def spawn_new_turtle(self):
    self.turtle_counter_ += 1
    name = self.turtle_name_perfix_ + str(self.turtle_counter_)
    x = random.uniform(0.0, 11.0)
    y = random.uniform(0.0, 11.0)
    theta = random.uniform(0.0, 2*math.pi)
    self.call_spawn_service(name, x, y, theta)
    
  def call_spawn_service(self, turtle_name, x, y, theta):
    while not self.spawn_client_.wait_for_service(1.0):
      self.get_logger().warn("Waiting for spawn service...")
    
    req = Spawn.Request()
    req.x = x
    req.y = y
    req.theta = theta
    req.name = turtle_name
    
    future = self.spawn_client_.call_async(req)
    future.add_done_callback(partial(self.callback_call_spawn, request=req))
  
  def callback_call_spawn(self, future, request):
    response: Spawn.Response = future.result()
    if response.name != "":
      self.get_logger().info("New alive turtle: " + response.name)
      new_turtle = Turtle()
      new_turtle.x = request.x
      new_turtle.y = request.y
      new_turtle.name = request.name
      new_turtle.theta = request.theta
      self.alive_turtles_.append(new_turtle)
      self.publish_alive_turtiles()

def main(args=None):
  rclpy.init(args=args)
  node = TurtleSpawnerNode()
  rclpy.spin(node)
  rclpy.shutdown()
  
if __name__ == "__main__":
  main()