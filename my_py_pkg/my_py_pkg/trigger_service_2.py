#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import Trigger
    
class TriggerService2(Node): 
    def __init__(self):
        super().__init__("trigger_service_2")
        self.count_ = -1
        self.trigger_server_ = self.create_service(Trigger, "trigger_two", self.callback_trigger_one)
        self.get_logger().info("Trigger 2 server has started")

    def callback_trigger_one(self, request: Trigger.Request, response: Trigger.Response):
        self.get_logger().info("Trigger 2 server called")
        self.count_ += 1
        response.success = True
        response.message = f"Trigger 2 response {self.count_}"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = TriggerService2() 
    rclpy.spin(node)
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()
    