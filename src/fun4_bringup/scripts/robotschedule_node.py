#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from fun4_interfaces.srv import Modeselect, Checkstate, Datapoint, Wantink, Qtarget, Gettarget

class RobotscheduleNode(Node):
    def __init__(self):
        super().__init__('robotschedule_node')

        """VALUE"""
        self.dt = 0.01
        self.mode_sate = 0
        self.controll_state = 1

        """SERVICE"""
        self.create_service(Modeselect, '/mode', self.Modeselect_callback)
        self.create_service(Datapoint, '/target_manual', self.Targetmanual_callback)  # Data Point

        """CLIENT"""
        self.check_state_cb = MutuallyExclusiveCallbackGroup()
        self.check_confroller_client = self.create_client(Checkstate, '/checkcontroller_state', callback_group=self.check_state_cb)
        self.sendq_target_cb = MutuallyExclusiveCallbackGroup()
        self.sendq_target_clinet = self.create_client(Qtarget, '/target_q', callback_group=self.sendq_target_cb)
        self.want_ink_clinet = self.create_client(Wantink, '/want_ink')
        self.gettarget_client = self.create_client(Gettarget,'/get_target')

        """TIME"""
        self.create_timer(self.dt,self.timer_loop)

        """Start node text"""
        self.get_logger().info(f'Starting: /{self.get_name()}')

    """Mode Select"""
    def Modeselect_callback(self, request, response):
        self.Check_controller_func()
        self.get_logger().info(f'Controller state is: {self.controll_state}')  # DEBUG
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
                self.get_logger().info(f'You select Mode "{request.modeselect}" -> no mode')
        else:
            response.success = False
            response.message = 'Controller is working now'
            self.get_logger().info('Controller is working now')
        return response
    
    """START CHECK CONTROLLER"""
    def Check_controller_func(self):
        msg = Checkstate.Request()
        msg.checkstate = True
        future = self.check_confroller_client.call_async(msg)
        future.add_done_callback(self.Check_controller_response)

    def Check_controller_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.controll_state = 1
            else:
                self.controll_state = 2
        except Exception as e:
            self.get_logger().error(f"Service call failed with error: {e}")

    """ END CHECK CONTROLLER """

    """ TARGET MANUAL CALLBACK """
    def Targetmanual_callback(self, request, response):
        self.get_logger().info(f"CALL TARGET MANUAL")  # DEBUG
        self.Check_controller_func()
        self.get_logger().info(f'controller is: {self.controll_state}')  # DEBUG

        if self.controll_state != 2:
            if self.mode_sate == 1:
                cal_target = Wantink.Request()
                cal_target.target.x = request.target.x
                cal_target.target.y = request.target.y
                cal_target.target.z = request.target.z
                self.get_logger().info(f'IPK GO TARGET: {cal_target.target}')  # DEBUG

                future_ink_result = self.want_ink_clinet.call_async(cal_target)
                future_ink_result.add_done_callback(lambda future: self.ink_result_response(future, response))

                response.success = True
                response.message = "we will fine kinematic for you"
                return response
            else:
                response.success = False
                response.message = 'Not in IPK Mode'
                return response
        else:
            response.success = False
            response.message = 'Controller is working'
            return response

    """ ink response """
    def ink_result_response(self, future, response):
        try:
            ink_result = future.result()

            self.get_logger().info(f'ink_result is: {ink_result}')  # DEBUG

            if ink_result.success:
                target_q = Qtarget.Request()
                target_q.q1 = ink_result.solution[0]
                target_q.q2 = ink_result.solution[1]
                target_q.q3 = ink_result.solution[2]

                self.sendq_target_clinet.call_async(target_q)

                response.success = True
                response.message = 'Yes, wait for it'
                return response
            else:
                response.success = False
                response.message = 'Failed to calculate IK'
                return response
        except Exception as e:
            self.get_logger().error(f'Service call to /want_ink failed: {e}')
            response.success = False
            response.message = f'Failed to calculate IK: {e}'
            return response
        
    """TIMER LOOP"""
    def timer_loop(self):
        if self.mode_sate == 3:
            self.get_logger().info("On mode 3 ") #DEBUG
            self.Check_controller_func()
            if self.controll_state == 1:
                self.get_logger().info("CONTROLL REDAY") #DEBUG
                self.Gettarget_fucn()
        pass

    """GET TARGET"""
    def Gettarget_fucn(self):
        msg = Gettarget.Request()
        msg.getdata = True
        futeru = self.gettarget_client.call_async(msg)
        futeru.add_done_callback(self.Gettarget_response)

    def Gettarget_response(self,future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"response is : {response}") #DEBUG
                cal_target = Wantink.Request()
                cal_target.target.x = response.xtarget
                cal_target.target.y = response.ytarget
                cal_target.target.z = response.ztarget
                self.get_logger().info(f'IPK GO TARGET: {cal_target.target}')  # DEBUG

                future_ink_result = self.want_ink_clinet.call_async(cal_target)
                future_ink_result.add_done_callback(lambda future: self.ink_result_response(future, response))
            else:
                # self.controll_state = 
                pass
        except Exception as e:
            self.get_logger().error(f"Service call failed with error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = RobotscheduleNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()