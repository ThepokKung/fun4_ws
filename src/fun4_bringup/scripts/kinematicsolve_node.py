#!/usr/bin/python3

import rclpy
from rclpy.node import Node

import roboticstoolbox as rtb
import numpy as np
from spatialmath import SE3
from fun4_interfaces.srv import Wantink

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
        self.create_service(Wantink,'/want_ink',self.Wantipk_callback)

        """Start node text"""
        self.get_logger().info(f'Starting {self.get_namespace()}/{self.get_name()}') 

    # def Find_ipk(self,x,y,z):
    #     goal_target = SE3(x,y,z)

    #     initial_joint_angles = np.random.uniform(low=-np.pi, high=np.pi, size=4)

    #     solutions = []
    #     best_solution = None
    #     min_norm = float('inf') 

    #     for i in range(10):
    #         solution = self.robot.ikine_LM(goal_target, q0=initial_joint_angles, mask=[1, 1, 1, 0, 0, 0])

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

    #     if best_solution is None:
    #         return [0.0, 0.0, 0.0, 0.0] 
    #     return best_solution 

    def Find_ipk(self, x, y, z):
        goal_target = SE3(x, y, z)
        
        solutions = []
        best_solution = None
        min_norm = float('inf')

        # Attempt to find a solution multiple times
        for i in range(10):
            # Random initial joint angles
            initial_joint_angles = np.random.uniform(low=-np.pi, high=np.pi, size=4)
            
            # Perform IK using the Levenberg-Marquardt solver
            solution = self.robot.ikine_LM(goal_target, q0=initial_joint_angles, mask=[1, 1, 1, 0, 0, 0])

            if solution.success:
                # Check if the solution is unique (avoid very similar solutions)
                is_unique = all(not np.allclose(solution.q, s, atol=1e-2) for s in solutions)
                
                if is_unique:
                    solutions.append(solution.q.tolist())

                    # Compute the norm relative to current joint configuration
                    norm = np.linalg.norm(np.array(solution.q) - np.array(self.q))

                    if norm < min_norm:
                        min_norm = norm
                        best_solution = solution.q.tolist()

        # Return the best solution, or a default value if none found
        return best_solution if best_solution is not None else [0.0, 0.0, 0.0, 0.0]
    
    def Wantipk_callback(self,request,response):
        if request is not None:
            response.success = True
            response.solution = self.Find_ipk(request.target.x,request.target.y,request.target.z)
        else:
            response.success = False
            response.solution = [0.0, 0.0, 0.0, 0.0] 
        return response
        
def main(args=None):
    rclpy.init(args=args)
    node = KinematicsSolverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__=='__main__':
    main()
