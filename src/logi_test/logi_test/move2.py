#!/usr/bin/env python 
import os
import sys
import getkey
import time

import rclpy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand


usage = """
Control Your OpenManipulator!
---------------------------
'1': Reset to initial position
'2': Execute grab-lift-place scenario
'q': Quit
"""

class Turtlebot3ManipulationTest(Node): 
    def __init__(self): 
        super().__init__('turtlebot3_manipulation_test')
        self.gripper_action_client = ActionClient(self, GripperCommand, 'gripper_controller/gripper_cmd')

        self.joint_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)

        # 초기화 포즈
        self.init_positions = [0.0, 0.0, 0.0, 0.0]  # Default initialization positions
        self.trajectory_msg = JointTrajectory()
        self.trajectory_msg.header = Header()
        self.trajectory_msg.header.frame_id = ''
        self.trajectory_msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

    def move_to_positions(self, positions, duration=5.0):
        """
        특정 조인트 위치로 로봇팔을 이동합니다.
        """
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)

        self.trajectory_msg.points = [point]
        self.joint_pub.publish(self.trajectory_msg)
        self.get_logger().info(f"Moving to positions: {positions}, duration: {duration}s")

    def reset_to_initial_position(self):
        """
        초기화 포즈로 이동합니다.
        """
        self.move_to_positions(self.init_positions, duration=5.0)
        
        
    #111111111111111#
    def ex1_1(self):
        lift_position = [0.35, 0.704597170656662, -0.42402399834829385, 1.3902231544865284]
        self.get_logger().info("2")
        self.move_to_positions(lift_position, duration=5.0)  
        
        
        
    #3333333333333333#
    def ex3_1(self):
        lift_position = [-0.3, 0.654597170656662, -0.4240239983482938, 1.3402231544865284]
        self.get_logger().info("2")
        self.move_to_positions(lift_position, duration=5.0) 
        
        
               
    #44444444444444#
    def ex4_1(self):
        lift_position = [0.44999999999999996, -0.045402829343338086, 0.7759760016517063, 0.6402231544865278]
        self.get_logger().info("2")
        self.move_to_positions(lift_position, duration=5.0)
    

        
    #666666666666666666#
    def ex6_1(self):
        # 밑으로 가서 잡기
        lift_position = [-0.4, -0.0045971706566619171, 0.87597600165170642, 0.4402231544865277]
        self.get_logger().info("2")
        self.move_to_positions(lift_position, duration=5.0)
        
    #적당한 위치#
    def initialpose(self):
        place_position = [0.0, -0.29540282934333806, 0.27597600165170616, 1.3902231544865284]
        self.get_logger().info("init")
        self.move_to_positions(place_position, duration=5.0)           
        
    # 내려놓기 ###############3
    def ex6_3(self):
        place_position = [-1.6000000000000008, 0.20459717065666194, 0.07597600165170619, 1.0402231544865281]
        self.get_logger().info("4")
        self.move_to_positions(place_position, duration=5.0)       
    
    def one_step(self):
        self.send_gripper_goal(0.025)
        self.initialpose()
        time.sleep(3)
        self.ex1_1()
        time.sleep(5)
        self.send_gripper_goal(-0.015)
        time.sleep(1)
        self.initialpose()
        time.sleep(5)
        self.ex6_3()
        time.sleep(6)
        self.send_gripper_goal(0.025)
        self.initialpose()
         
    def timer_callback(self):
        print('.')

        if self.gripper_state == 0:
            self.send_gripper_goal(0.025)  # Open
            self.gripper_state = 1
        else:
            self.send_gripper_goal(-0.015)  # Close
            self.gripper_state = 0

    def send_gripper_goal(self, position):
    # """Gripper goal 설정"""
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = -1.0

        if not self.gripper_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("Gripper action server not available!")
            return  

        self.gripper_action_client.send_goal_async(goal)
        
        
    def wait_for_duration(self, duration):
        """
        지정된 시간 동안 대기합니다.
        """
        self.get_logger().info(f"Waiting for    {duration} seconds...")
        time.sleep(duration)


def main(args=None):
    rclpy.init()
    node = Turtlebot3ManipulationTest()
    

    print(usage)
    try:
        while rclpy.ok():
            key_value = getkey.getkey()

            if key_value == 'i':
                node.reset_to_initial_position()
                
            elif key_value == '1':
                node.send_gripper_goal(0.025)
                node.initialpose()
                time.sleep(3)
                node.ex1_1()
                time.sleep(5)
                node.send_gripper_goal(-0.015)
                time.sleep(1)
                node.initialpose()
                time.sleep(5)
                node.ex6_3()
                time.sleep(6)
                node.send_gripper_goal(0.025)
                node.initialpose()
                
            elif key_value == '3':
                node.send_gripper_goal(0.025)
                node.initialpose()
                time.sleep(3)
                node.ex3_1()
                time.sleep(5)
                node.send_gripper_goal(-0.015)
                time.sleep(1)
                node.initialpose()
                time.sleep(5)
                node.ex6_3()
                time.sleep(6)
                node.send_gripper_goal(0.025)
                node.initialpose()


            
            elif key_value == '6':
                node.send_gripper_goal(0.025)
                node.initialpose()
                time.sleep(3)
                node.ex6_1()
                time.sleep(5)
                node.send_gripper_goal(-0.015)
                time.sleep(1)
                node.initialpose()
                time.sleep(5)
                node.ex6_3()
                time.sleep(6)
                node.send_gripper_goal(0.025)
                node.initialpose()
                
                
                
            elif key_value == 'o':
                node.send_gripper_goal(0.025)
            elif key_value == 'p':
                node.send_gripper_goal(-0.015)
            elif key_value == '0':
                node.initialpose()
            
            elif key_value == 'q':
                break

    except Exception as e:
        print(f"Error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__": 
    main()
