#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from fun4_interfaces.srv import Checkstate
from ament_index_python.packages import get_package_share_directory
import os, random
import numpy as np

class RandomtargetposeNode(Node):
    def __init__(self):
        super().__init__('randomtargetpose_node')

        """PUB"""
        self.target_pub = self.create_publisher(PoseStamped,'/target',10) 

        """CSV PATH"""
        csv_file_path = os.path.join(
            get_package_share_directory('fun5_bringup'),
            'config',
            'workspace_points.csv'
        )
        self.workspace_points = np.loadtxt(csv_file_path, delimiter=',')

        """Clinet"""
        self.checktargetdone_client =self.create_client(Checkstate,'/targetdone_state')

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
            self.targetpub_func(x_random,y_random,z_random)
            self.get_logger().info(response.message)
        else:
            response.success = False
            response.message = "Notthing ....."
        return response
    
    def targetpub_func(self,x,y,z):
        msg = PoseStamped()
        msg.header.frame_id = 'link_0'
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.position.x = x /1000
        msg.pose.position.y = y /1000
        msg.pose.position.z = z /1000

        self.get_logger().info(f'Pub target data x :{msg.pose.position.x}, y : {msg.pose.position.y},z : {msg.pose.position.z}')
        self.target_pub.publish(msg)

    def Targetdone_callback(self,request,response):
        if request.checkstate == True:
            response.success = True
            response.message = "Now i know you done"
            self.get_logger().info(response.message)
        else:
            response.success = False
            response.message = "Notthing..."
            self.get_logger().info(response.message)
        return response
    
def main(args=None):
    rclpy.init(args=args)
    node = RandomtargetposeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
