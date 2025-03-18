#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from time import sleep

from my_robot_interfaces.srv import SetLed
    
    
class BatteryNode(Node):
    def __init__(self):
        super().__init__("battery")
        self.battery_status_on = True
        self.get_logger().info("Battery has been started.")
        
        while True:
            self.discharge_battery()
            self.charge_battery()
    
    def discharge_battery(self):
        sleep(4)
        self.battery_status_on = False
        self.get_logger().info("Battery is empty.")
        self.battery_client(0, True)
    
    def charge_battery(self):
        sleep(6)
        self.battery_status_on = True
        self.get_logger().info("Battery is full.")
        self.battery_client(0, False)
    
    def battery_client(self, led_number, state):
        client = self.create_client(SetLed, "set_led")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Set Led")
        
        request = SetLed.Request()
        request.led_number = led_number
        request.state = state
        
        future = client.call_async(request)
        future.add_done_callback(self.callback_set_led_state)

    def callback_set_led_state(self, future):
        response = future.result()
        if response.success:
            self.get_logger().info("LED state has changed.")
        else:
            self.get_logger().info("LED state not changed.")
            
def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode()
    rclpy.spin(node)
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()
