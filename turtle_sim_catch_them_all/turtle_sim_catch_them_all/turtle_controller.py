#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from math import atan2, pi
from functools import partial

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from my_robot_interfaces.msg import TurtleArray
from my_robot_interfaces.srv import CatchTurtle
    
    
class TurtleControllerNode(Node): 
    def __init__(self):
        super().__init__("turtle_controller")
        self.declare_parameter("catch_closest_turtle_first", True)
        self.catch_closest_turtle_first = self.get_parameter("catch_closest_turtle_first").value
        self.subcriber_ = self.create_subscription(Pose, "/turtle1/pose", self.callback_turtle_pose, 10)
        self.publish_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.pose_ = Pose()
        self.catch_turtle = None
        self.alive_turtles_subscriber = self.create_subscription(TurtleArray, "/alive_turtles", self.callback_aliver_turtles, 10)

        self.timer = self.create_timer(0.1, self.publish_move)
        self.get_logger().info("Turtle Controller Node has been started.")
    
    def callback_aliver_turtles(self, msg):
        if len(msg.turtle_array) > 0:
            if self.catch_closest_turtle_first:
                closest_turtle = None
                closest_distance = None
                for turtle in msg.turtle_array:
                    dist_x = turtle.x - self.pose_.x
                    dist_y = turtle.y - self.pose_.y
                    dist = ((dist_x)**2 + (dist_y)**2)**(0.5)
                    if closest_turtle == None or dist < closest_distance:
                        closest_turtle = turtle
                        closest_distance = dist
                self.catch_turtle = closest_turtle
            else:
                self.catch_turtle = msg.turtle_array[0]
        
    def callback_turtle_pose(self, msg):
        self.pose_ = msg    
    
    def move_to(self, x: float, y: float, name: str):
        dist_x = x - self.pose_.x
        dist_y = y - self.pose_.y
        if abs(dist_x) > 0.5 or abs(dist_y) > 0.5:
            distance = ((dist_x)**2 + (dist_y)**2)**(0.5)
            angle = atan2(dist_y,dist_x) - self.pose_.theta
            if angle > pi:
                angle -= 2*pi
            elif angle < -pi:
                angle += 2*pi
            return 2*distance, 6*angle
        else:
            self.catch_turtle_client(name)
            return (0.0, 0.0)
    
    def publish_move(self):
        msg = Twist()
        if self.catch_turtle != None:
            msg.linear.x, msg.angular.z = self.move_to(self.catch_turtle.x, self.catch_turtle.y, self.catch_turtle.name)
            self.publish_.publish(msg)

    def catch_turtle_client(self, name: str):
        client = self.create_client(CatchTurtle, "catch_turtle")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Catch Turtle")
        
        request = CatchTurtle.Request()
        request.turtle_name = name

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_catch_turtle, request=request))
    
    def callback_catch_turtle(self, future, request):
        try:
            response = future.result()
            self.get_logger().info(f"Turtle {request.turtle_name} kill msg send")
            self.catch_turtle = None
        except Exception as e:
            self.get_logger().error(f"Service call failed {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode() 
    rclpy.spin(node)
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()
