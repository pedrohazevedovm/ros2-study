#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from my_robot_interfaces.msg import LedPanelState
from my_robot_interfaces.srv import SetLed
    
class LedNode(Node):
    def __init__(self):
        super().__init__("led")
        self.declare_parameter("led_status", [0, 0, 0])
        self.led_status = self.get_parameter("led_status").value
        self.server_ = self.create_service(SetLed, "set_led", self.callback_set_led)
        self.publisher_ = self.create_publisher(LedPanelState, "led_panel_state", 10)
        self.timer_ = self.create_timer(1.0, self.publisher_led_state)
        self.get_logger().info("Led State has been started.")
    
    def callback_set_led(self, request: SetLed.Request, response: SetLed.Response):
        led_number = request.led_number
        state = request.state
        
        if led_number >= len(self.led_status) or led_number < 0:
            response.success = False
            return response
        
        if state:
            self.led_status[led_number] = 1
        else:
            self.led_status[led_number] = 0
        response.success = True
        return response
    
    def publisher_led_state(self):
        msg = LedPanelState()
        msg.led_state = self.led_status
        self.publisher_.publish(msg)
    
def main(args=None):
    rclpy.init(args=args)
    node = LedNode()
    rclpy.spin(node)
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()
    