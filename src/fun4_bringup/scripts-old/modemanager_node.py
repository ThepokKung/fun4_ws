#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from fun4_interfaces.srv import Modeselect, Checkstate, Setstate

"""
Mode state
0 is not ready
1 is Inverse Pose Kinematics Mode (IPK)
2 is Tele-operation Mode (Teleop)
3 is Autonomous Mode (Auto)
"""
"""
Controll State
1 is ready
2 is working
"""

class ModeManagerNode(Node):
    def __init__(self):
        super().__init__('modemanager_node')

        """VALUE"""
        self.mode_sate = 0
        self.controll_state = 1

        """SERVICE"""
        self.create_service(Modeselect,'/mode',self.Modeselect_callback)

        """CLIENT"""
        self.ipk_mode_client = self.create_client(Setstate,'/setipk_mode')

    def Modeselect_callback(self, request, response):
        if self.controll_state != 2:
            if request.modeselect == 1:
                self.mode_sate = 1
                self.Setipk_mode(True)
                response.success = True
                response.message = 'You select Mode "1" -> Inverse Pose Kinematics Mode (IPK)'
                self.get_logger().info('You select Mode "1" -> Inverse Pose Kinematics Mode (IPK)')
            elif request.modeselect == 2:
                self.mode_sate = 2
                response.success = True
                response.message = 'You select Mode "2" -> Tele-operation Mode (Teleop)'
                self.get_logger().info('You select Mode "2" -> Tele-operation Mode (Teleop)')
            elif request.modeselect == 3:
                self.mode_sate = 3
                response.success = True
                response.message = 'You select Mode "3" -> Autonomous Mode (Auto)'
                self.get_logger().info('You select Mode "3" -> Autonomous Mode (Auto)')
            else:
                response.success = False
                response.message = f'You select Mode "{request.modeselect}" -> no mode'
                self.get_logger().info(f'You select Mode "{request.modeselect}" -> no mode' )
        else:
            response.success = False
            response.message = 'Controller is working now'
            self.get_logger().info('Controller is working now')
        return response
    
    def Setipk_mode(self,on):
        msg = Setstate.Request()
        msg.setstate = on
        self.ipk_mode_client.call_async(msg)
    
def main(args=None):
    rclpy.init(args=args)
    node = ModeManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
