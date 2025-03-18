#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from time import sleep

from my_robot_interfaces.srv import Trigger
    
class TriggerClientNode(Node): 
    def __init__(self):
        super().__init__("trigger_client")
        self.client_ = self.create_client(Trigger, "trigger_one")
        self.timer_ = self.create_timer(0.5, self.trigger_one_server)

    def trigger_one_server(self):
        while not self.client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Trigger One Server...")
        
        request = Trigger.Request()

        future = self.client_.call_async(request)
        future.add_done_callback(self.callback_trigger_one)
    
    def callback_trigger_one(self, future: Trigger.Response):
        try:
            response = future.result()
            self.get_logger().info(f"Response Success: {response.message}!")
        except Exception as e:
            self.get_logger().error(f"Sevice call failed {e}")
    
    
def main(args=None):
    rclpy.init(args=args)
    node = TriggerClientNode()
    rclpy.spin(node)
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()
    