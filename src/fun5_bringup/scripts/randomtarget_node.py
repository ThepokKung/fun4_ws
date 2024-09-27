#!/usr/bin/env python3

import yaml.loader
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from fun5_interfaces.srv import Gettarget
from ament_index_python.packages import get_package_share_directory
import os, yaml, random
import numpy as np

class RandomtargetposeNode(Node):
    def __init__(self):
        super().__init__('randomtargetpose_node')
        

        """CSV PATH"""
        csv_file_path = os.path.join(
            get_package_share_directory('fun5_bringup'),
            'config',
            'workspace_points.csv'
        )
        self.workspace_points = np.loadtxt(csv_file_path, delimiter=',')

        """SERVICE"""
        self.create_service(Gettarget,'/get_target',self.Gettarget_callback)

        """Start node text"""
        self.get_logger().info(f'Starting {self.get_namespace()}/{self.get_name()}') 

    def Gettarget_callback(self, request, response):
        if request.getdata == True:
            random_row = random.choice(self.workspace_points)
            x_random, y_random, z_random = random_row
            response.xtarget = float(x_random)
            response.ytarget = float(y_random)
            response.ztarget = float(z_random)
            response.success = True
            response.message = f'get target : x : {x_random}, y : {y_random}, z : {z_random}'
        else:
            response.success = False
            response.message = "Notthing ....."
        return response

    
def main(args=None):
    rclpy.init(args=args)
    node = RandomtargetposeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
