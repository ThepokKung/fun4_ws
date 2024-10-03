#!/usr/bin/python3

import rclpy
from rclpy.node import Node

from fun4_interfaces.srv import Checkstate


class AutoNode(Node):
    def __init__(self):
        super().__init__('monde_auto_node')

        """CLIENT"""
        self.check_controller_client = self.create_client(Checkstate,'/checkcontroller_state',callback_group=self.call_state_cb)

def main(args=None):
    rclpy.init(args=args)
    node = AutoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
