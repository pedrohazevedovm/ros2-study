#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import Trigger
    
class TriggerService1(Node): 
    def __init__(self):
        super().__init__("trigger_service_1")
        self.trigger_server_ = self.create_service(Trigger, "trigger_one", self.callback_trigger_one)
        self.trigger2_client = self.create_client(Trigger, "trigger_two")
        self.get_logger().info("Trigger 1 server has started")
        self.message_ = "not yet"
        self.success_ = False

    def callback_trigger_one(self, request: Trigger.Request, response: Trigger.Response):
        self.get_logger().info("Trigger 1 server called")
        self.call_trigger_2_server()
        response.success = self.success_
        response.message = self.message_
        return response

    def call_trigger_2_server(self):
        while not self.trigger2_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Trigger 2 Server...")
        
        request = Trigger.Request()

        future = self.trigger2_client.call_async(request)
        future.add_done_callback(self.callback_trigger_2)
    
    def callback_trigger_2(self, future: Trigger.Response):
        try:
            response = future.result()
            self.get_logger().info(f"Response Success: {response.message}!")
            self.message_ = response.message
            self.success_ = response.success            
        except Exception as e:
            self.get_logger().error(f"Sevice call failed {e}")
    

def main(args=None):
    rclpy.init(args=args)
    node = TriggerService1() 
    rclpy.spin(node)
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()
    