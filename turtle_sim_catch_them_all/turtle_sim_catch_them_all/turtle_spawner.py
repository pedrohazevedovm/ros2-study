#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import random
from functools import partial

from turtlesim.srv import Spawn, Kill
from my_robot_interfaces.msg import Turtle, TurtleArray
from my_robot_interfaces.srv import CatchTurtle
    
class TurtleSpawnerNode(Node):
    def __init__(self):
        super().__init__("turtle_spawner")
        self.declare_parameter("spawn_frequency", 1.0)
        self.turtle_list = []
        self.spawn_frequency = self.get_parameter("spawn_frequency").value
        self.spawn_timer = self.create_timer(1/self.spawn_frequency, self.spawner_turtle_client)
        self.alive_turltes_publisher = self.create_publisher(TurtleArray, "/alive_turtles", 10)
        
        self.alive_turtles_timer = self.create_timer(0.1, self.callback_alive_turtles)
        self.catch_turtle_server = self.create_service(CatchTurtle, "catch_turtle", self.callback_catch_turtle)
    
    def callback_catch_turtle(self, request, response):
        turtle_to_kill = request.turtle_name
        for turtle in self.turtle_list:
            if turtle[2] == turtle_to_kill:
                self.turtle_list.remove(turtle)
                self.kill_turtle_client(turtle_to_kill)
                return response
        return response

    def callback_alive_turtles(self):
        msg = TurtleArray()
        turtle_array = []
        if len(self.turtle_list) > 0:
            for turtle in self.turtle_list:
                alive_turtle = Turtle()
                alive_turtle.x, alive_turtle.y, alive_turtle.name = turtle
                turtle_array.append(alive_turtle)
        msg.turtle_array = turtle_array
        self.alive_turltes_publisher.publish(msg)
        
    def spawner_turtle_client(self):
        client = self.create_client(Spawn, "spawn")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Spawn.")
        
        request = Spawn.Request()
        request.x = random.uniform(0, 11)
        request.y = random.uniform(0, 11)
        request.theta = 0.0

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_spawn_turtle, request=request))
    
    def callback_spawn_turtle(self, future, request):
        try:
            response = future.result()
            self.get_logger().info(response.name)
            self.turtle_list.append([request.x, request.y, response.name])
        except Exception as e:
            self.get_logger().error(f"Service call failed {e}")
    
    def kill_turtle_client(self, name: str):
        client = self.create_client(Kill, "kill")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting dor Server Kill")
        
        request = Kill.Request()
        request.name = name
        
        future = client.call_async(request)
        future.add_done_callback(self.callback_kill_turtle)

    def callback_kill_turtle(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f"Service call failed {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawnerNode()
    rclpy.spin(node)
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()
    