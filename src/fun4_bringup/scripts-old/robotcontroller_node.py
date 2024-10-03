#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from fun4_interfaces.srv import Qtarget,Getq,Checkstate

class RobotControllerNode(Node):
    def __init__(self):
        super().__init__('robotcontroller_node')

        """PUB"""
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)

        """STATE"""
        self.controller_state = 1
        self.request_target = False

        """ROBOT STATE"""
        self.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4"]
        self.q = [0.0, 0.5, 1.4, 0.0]
        self.target_q = [0.0, 0.0, 0.0, 0.0]

        """ROBOT VALUE"""
        self.max_w = 1.5
        self.dt = 0.01

        """TIMER"""
        self.create_timer(self.dt, self.sim_loop)

        """SERVICE"""
        self.create_service(Qtarget,'/target_q',self.Targetq_callback)
        self.create_service(Getq,'/get_q',self.Getq_callback)
        self.create_service(Checkstate,'/checkcontroller_state',self.Checkcontroller_callback)

        """START TEXT"""
        self.get_logger().info('Robot Controller Node started')

    def sim_loop(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        for i in range(len(self.q)):
            delta_q = self.target_q[i] - self.q[i]
            if abs(delta_q) > (self.max_w * self.dt):
                self.q[i] += self.max_w * self.dt if delta_q > 0 else -self.max_w * self.dt
            else:
                self.q[i] = self.target_q[i]

            msg.position.append(self.q[i])
            msg.name.append(self.joint_names[i])

        if all(abs(self.q[i] - self.target_q[i]) <= 0.1 for i in range(len(self.q))):
            if self.controller_state == 2 :
                self.controller_state = 1

        self.joint_pub.publish(msg)

    def Targetq_callback(self,request,response):
        if request is not None and self.controller_state == 1:
            self.target_q[0] = request.q1 
            self.target_q[1] = request.q2
            self.target_q[2] = request.q3
            self.controller_state = 2
            response.success = True
            response.message = f'Set q to : {self.target_q[:3]}'
            self.get_logger().info(response.message)
        return response
    
    def Getq_callback(self,request,response):
        if request.giveqforme == True:
            response.success = True
            response.message = f'Give Q : {self.q[:3]}'
            response.q1 = self.q[0]
            response.q2 = self.q[1]
            response.q3 = self.q[2]
        else:
            response.success = False
            response.message = 'Not give for you'
            response.q1 = 0.0
            response.q2 = 0.0
            response.q3 = 0.0
        return response
    
    def Checkcontroller_callback(self,request,response):
        if request.checkstate:
            if self.controller_state == 2:
                response.success = False
                response.message = 'This controller is working'
            elif self.controller_state == 1:
                response.success = True
                response.message = 'This controller is not working'
        else:
            response.success = False
            response.message = 'This controller is working'
        self.get_logger().info(f'Check Controlelr State : {response}') #DEBUG
        return response

def main(args=None):
    rclpy.init(args=args)
    node = RobotControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
