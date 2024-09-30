#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import roboticstoolbox as rtb
import numpy as np

from fun5_interfaces.srv import Modeselect,Datapoint,Checkstate,Gettarget
from geometry_msgs.msg import PoseStamped
from spatialmath import *
from sensor_msgs.msg import JointState

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

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        """PUB"""
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.endeffector_pub = self.create_publisher(PoseStamped, "/end_effector", 10)  

        """VALUE"""
        self.mode_sate = 0
        self.controll_state = 1
        self.requesttarget_state = False

        """Service"""
        self.create_service(Modeselect,'/mode',self.Modeselect_callback) #Mode Secect Service
        self.create_service(Datapoint,'/target_mannal',self.Datapoint_callback) #Data Point
        self.create_service(Checkstate,'/checkcontroller_state',self.Checkcontrollerstate_callback)
        self.create_service(Checkstate,'/checkmode_state',self.Checkmode_callback)
        
        """Clinet"""
        self.gettarget_client = self.create_client(Gettarget,'/get_target')

        """ROBOT"""
        self.robot = rtb.DHRobot([
            rtb.RevoluteMDH(d=200),
            rtb.RevoluteMDH(d=-120, alpha=-np.pi/2,offset=-np.pi/2),
            rtb.RevoluteMDH(d=100, a=250, alpha=0,offset=np.pi/2),
            rtb.RevoluteMDH(d=280,alpha=np.pi/2)
        ], name='rrr_robot')

        """ROBOT JOINT VAL"""
        self.name = ["joint_1", "joint_2", "joint_3", "joint_4"]
        self.q = [0.0, 0.5, 1.4,0.0]
        self.target_q = [0.0,0.0,0.0,0.0]
        self.max_w = 1.5

        """ROBOT TIMER"""
        self.dt = 0.01
        self.create_timer(self.dt,self.sim_loop)

        """Start node text"""
        self.get_logger().info(f'Starting {self.get_namespace()}/{self.get_name()}') 

    def sim_loop(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        for i in range(len(self.q)):

            if self.mode_sate == 3 and self.controll_state == 1:
                self.Gettatget_client_func()
                self.requesttarget_state = True
            
            delta_q = self.target_q[i] - self.q[i]

            if abs(delta_q) > (self.max_w * self.dt):
                if delta_q > 0:
                    self.q[i] += self.max_w * self.dt
                else:
                    self.q[i] -= self.max_w * self.dt
            else:
                self.q[i] = self.target_q[i]

            msg.position.append(self.q[i])
            msg.name.append(self.name[i])

        if all(abs(self.q[i] - self.target_q[i]) <= 0.1 for i in range(len(self.q))):
            self.controll_state = 1

        self.joint_pub.publish(msg)
        self.Endeffector_pub_func()

    def Find_ikine(self,x,y,z):
        goal_pose = SE3(x,y,z)

        solutions = []
        best_solution = None
        min_norm = float('inf') 

        for i in range(10):
            initial_joint_angles = np.random.uniform(low=-np.pi, high=np.pi, size=4)
    
            solution = self.robot.ikine_LM(goal_pose, q0=initial_joint_angles, mask=[1, 1, 1, 0, 0, 0])

            if solution.success:
                is_unique = True
                for s in solutions:
                    if np.allclose(solution.q, s, atol=1e-2):  
                        is_unique = False
                        break
                    
                if is_unique:
                    solutions.append(solution.q.tolist())
                    norm = np.linalg.norm(np.array(solution.q) - np.array(self.q))

                    if norm < min_norm:
                        min_norm = norm
                        best_solution = solution.q.tolist()

        return best_solution 
    
    def Endeffector_pub_func(self):
        end_effector_pose = self.robot.fkine(self.q)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "link_0"

        msg.pose.position.x = end_effector_pose.t[0] /1000
        msg.pose.position.y = end_effector_pose.t[1] /1000
        msg.pose.position.z = end_effector_pose.t[2] /1000

        self.endeffector_pub.publish(msg)
    
    def targetpub_func(self,x,y,z):
        msg = PoseStamped()
        msg.header.frame_id = 'link_0'
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.position.x = x /1000
        msg.pose.position.y = y /1000
        msg.pose.position.z = z /1000

        self.get_logger().info(f'Pub target data x :{msg.pose.position.x}, y : {msg.pose.position.y},z : {msg.pose.position.z}')
        self.target_pub.publish(msg)

    def Gettatget_client_func(self):
        command_request = Gettarget.Request()
        command_request.getdata = True
        future = self.gettarget_client.call_async(command_request)
        future.add_done_callback(lambda future: self.Gettarget_response_callback(future))

    def Gettarget_response_callback(self, future):
        try:
            response = future.result()
            if self.mode_sate == 3 and self.controll_state == 1:
                if response.success:
                    x_data = response.xtarget
                    y_data = response.ytarget
                    z_data = response.ztarget
                    find_ikine = self.Find_ikine(x_data,y_data,z_data)
                    self.get_logger().info(f'Get data position x : {x_data}, y : {y_data}, z : {z_data}')
                    if  find_ikine != None:
                        self.controll_state = 2
                        self.target_q = find_ikine
                        self.get_logger().info('We can go to postition wait for going')
                        self.targetpub_func(x_data, y_data, z_data)
                else:
                    pass
            else:
                pass
        except Exception as e:
            self.get_logger().error(f"Service call failed with error: {e}")

    def Modeselect_callback(self, request, response):
        if self.controll_state != 2:
            if request.modeselect == 1:
                self.mode_sate = 1
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
    
    def Datapoint_callback(self,request,response):
        if self.mode_sate == 1 and self.controll_state == 1:
            x_data = request.xgoal
            y_data = request.ygoal
            z_data = request.zgoal
            find_ikine = self.Find_ikine(x_data,y_data,z_data)
            self.get_logger().info(f'Get data position x : {x_data}, y : {y_data}, z : {z_data}')
            if  find_ikine != None:
                response.success = True
                response.message = "We can go to postition wait for going"
                self.controll_state = 2
                self.target_q = find_ikine
                self.get_logger().info('We can go to postition wait for going')
            else:
                response.success = False
                response.message = "We can't go to postition sorry"
                self.get_logger().info("We can't go to postition sorry")
        else:
            response.success = False
            response.message = "Mode state don't match or Working now"
            self.get_logger().info("Mode state don't match or Working now")
        return response
    
    def Checkcontrollerstate_callback(self,request,response):
        if request.checkstate == True:
            response.success = True
            response.message = f'Now robot state is {self.controll_state}'
            self.get_logger().info(f'Now robot state is {self.controll_state}')
        return response
    
    def Checkmode_callback(self,request,response):
        if request.checkstate == True:
            response.success = True
            response.message = f'Now mode state is {self.mode_sate}'
            self.get_logger().info(f'Now mode state is {self.mode_sate}')
        return response
        
def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
