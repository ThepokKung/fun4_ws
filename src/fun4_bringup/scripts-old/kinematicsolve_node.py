#!/usr/bin/python3

import rclpy
from rclpy.node import Node

import roboticstoolbox as rtb
import numpy as np
from spatialmath import SE3
from fun4_interfaces.srv import Wantink,Getq

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


class KinematicsSolverNode(Node):
    def __init__(self):
        super().__init__('kinematics_solver_node')

        """ROBOT"""
        self.robot = rtb.DHRobot([
            rtb.RevoluteMDH(d=200),
            rtb.RevoluteMDH(d=-120, alpha=-np.pi/2,offset=-np.pi/2),
            rtb.RevoluteMDH(d=100, a=250, alpha=0,offset=np.pi/2),
            rtb.RevoluteMDH(d=280,alpha=np.pi/2)
        ], name='rrr_robot')

        """ROBOT JOINT VAL"""
        self.name = ["joint_1", "joint_2", "joint_3", "joint_4"]
        self.target_q = [0.0,0.0,0.0,0.0]
        self.q = [0.0, 0.5, 1.4,0.0]
        self.max_w = 1.5

        """ROBOT TIMER"""
        self.dt = 0.01

        """SERVICE"""
        self.create_service(Wantink,'/want_ink',self.Wantink_callback)

        """CLIENT"""
        self.get_q_cb = MutuallyExclusiveCallbackGroup()
        self.getq_clinet = self.create_client(Getq,'/get_q',callback_group=self.get_q_cb)

        """Start node text"""
        self.get_logger().info(f'Starting {self.get_namespace()}/{self.get_name()}') 

    # def Find_ink(self, x, y, z):
    #     goal_target = SE3(x, y, z)
        
    #     self.Getq_func()

    #     # msg = Getq.Request()
    #     # msg.giveqforme = True
    #     # result = self.getq_clinet.call(msg)

    #     # if result.success == True:
    #     #     self.q[0] = result.q1
    #     #     self.q[1] = result.q2
    #     #     self.q[2] = result.q3
    
    #     solutions = []
    #     best_solution = None
    #     min_norm = float('inf')

    #     for i in range(10):
    #         initial_joint_angles = np.random.uniform(low=-np.pi, high=np.pi, size=4)
            
    #         solution = self.robot.ikine_LM(goal_target, q0=initial_joint_angles, mask=[1, 1, 1, 0, 0, 0])

    #         if solution.success:
    #             is_unique = all(not np.allclose(solution.q, s, atol=1e-2) for s in solutions)
                
    #             if is_unique:
    #                 solutions.append(solution.q.tolist())

    #                 norm = np.linalg.norm(np.array(solution.q) - np.array(self.q))

    #                 if norm < min_norm:
    #                     min_norm = norm
    #                     best_solution = solution.q.tolist()

    #     return best_solution if best_solution is not None else [0.0, 0.0, 0.0, 0.0]

    # def Find_ikine(self,x,y,z):
    #     goal_pose = SE3(x,y,z)

    #     self.Getq_func()

    #     solutions = []
    #     best_solution = None
    #     min_norm = float('inf') 

    #     for i in range(10):
    #         initial_joint_angles = np.random.uniform(low=-np.pi, high=np.pi, size=4)
    
    #         solution = self.robot.ikine_LM(goal_pose, q0=initial_joint_angles, mask=[1, 1, 1, 0, 0, 0])

    #         if solution.success:
    #             is_unique = True
    #             for s in solutions:
    #                 if np.allclose(solution.q, s, atol=1e-2):  
    #                     is_unique = False
    #                     break
                    
    #             if is_unique:
    #                 solutions.append(solution.q.tolist())
    #                 norm = np.linalg.norm(np.array(solution.q) - np.array(self.q))

    #                 if norm < min_norm:
    #                     min_norm = norm
    #                     best_solution = solution.q.tolist()

    #     return best_solution 

    def Find_ikine(self,x,y,z):
        goal_target = SE3(x,y,z)

        # self.Getq_func()
        initial_joint_angles = np.random.uniform(low=-np.pi, high=np.pi, size=4)

        solutions = []
        best_solution = None
        min_norm = float('inf') 

        for i in range(10):
            solution = self.robot.ikine_LM(goal_target, q0=initial_joint_angles, mask=[1, 1, 1, 0, 0, 0])

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

        if best_solution is None:
            return [0.0, 0.0, 0.0, 0.0] 
        return best_solution 
    
    def Wantink_callback(self,request,response):
        if request is not None:
            temp = self.Find_ikine(request.target.x,request.target.y,request.target.z)
            if temp != [0.0, 0.0, 0.0, 0.0]:
                response.success = True
                response.solution = temp
                self.get_logger().info(f"We find solove : {temp}")
            else:
                response.success = False
                response.solution = temp
                self.get_logger().info(f"We can't find solove")
        else:
            response.success = False
            response.solution = [0.0, 0.0, 0.0, 0.0] 
            self.get_logger().info(f"We can't find solove")
        return response
    
    def Getq_func(self):
        msg = Getq.Request()
        msg.giveqforme = True
        result = self.getq_clinet.call(msg)

        if result.success == True:
            self.q[0] = result.q1
            self.q[1] = result.q2
            self.q[2] = result.q3
        
def main(args=None):
    rclpy.init(args=args)
    node = KinematicsSolverNode()
    # rclpy.spin(node)

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()
if __name__=='__main__':
    main()
