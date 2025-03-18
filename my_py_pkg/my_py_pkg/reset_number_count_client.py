#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from functools import partial

from example_interfaces.srv import SetBool

    
class ResetNumberCountClientNode(Node): 
    def __init__(self):
        super().__init__("reset_number_count_client")
        self.reset_count_server(True)
    
    def reset_count_server(self, data):
        client = self.create_client(SetBool, "reset_counter")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Reset Number Count")
        
        request = SetBool.Request()
        request.data = data
        
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_reset_number_count, data=data))
    
    def callback_call_reset_number_count(self, future, data):
        try:
            response = future.result()
            self.get_logger().info(response.message)
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))
    
    
def main(args=None):
    rclpy.init(args=args)
    node = ResetNumberCountClientNode() 
    rclpy.spin(node)
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()
    