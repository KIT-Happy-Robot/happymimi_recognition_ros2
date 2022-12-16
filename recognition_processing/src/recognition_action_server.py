
import rclpy
from rclpy.node import Node

import os
import sys
import time
import math
import numpy
from rclpy.action import ActionServer
import smach
from smach
from smach_ros import ActionServerWrapper
from geometry_msgs.msg import Twist, Point

from darknet_ros_msgs.msg import BoundingBoxes
from happymimi_msgs.msg import StrInt
from happymimi_recognition_msgs.msg import RecognitionProcessingAction, RecognitionProcessingResult
from happymimi_recognition_msgs.srv import RecognitionCountRequest, RecognitionFindRequest, RecognitionLocalizeRequest

from recognition_tools import RecognitionTools
teleop_path = roslib.packages.get_pkg_dir('happymimi_teleop')
sys.path.insert(0, os.path.join(teleop_path, 'src/'))
from base_control import BaseControl


class Server(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['start_action'],
                             input_keys = ['goal_in'],
                             output_keys = ['target_name_out', 'sort_option_out', 'e_l_count_out', 'c_l_count_out', 'move_count_out', 'result_out'])
        self.counter = 0
        self.logger = _node.get_logger()

    def execute(self, userdata):
        self.logger.info('Executing state: Server')

        userdata.target_name_out = userdata.goal_in.target_name
        userdata.sort_option_out = userdata.goal_in.sort_option
        userdata.e_l_count_out = 0
        userdata.c_l_count_out = 0
        userdata.move_count_out = 0
        userdata.result_out = RecognitionProcessingResult(result_flg=False, centroid_point=Point())
        return 'start_action'
        

class Count(smach.State):
    def __init__(self):

    def execute(self, userdata):


class Find(smach.State):
    def __init__(self):

    def execute(self, userdata):


class Localize(smach.State):
    def __init__(self):

    def execute(self, userdata):

class CheckCenter(smach.State):
    def __init__(self):

    def execute(self, userdata):

class Move(smach.State):
    def __init__(self):

    def execute(self, userdata):

if __name__ == '__main__':
    rclpy.init()
    node = Node('recognition_action_server')

    global Recognition_Tools
    Recognition_Tools = RecognitionTools()

    sm = smach.StateMachine(outcomes = ['success', 'action_failed', 'preemted'],
                            input_keys = ['action_goal', 'action_result'],
                            output_keys = ['action_result'])

    with sm:
        smach.StateMachine.add('SERVER', Server(),
                               transitions = {'start_action':'COUNT'},
                               remapping = {'goal_in':'action_goal',
                                            'target_name_out':'target_name',
                                            'sort_option_out':'sort_option',
                                            'e_l_count_out':'existence_loop_count',
                                            'c_l_count_out':'center_loop_count',
                                            'move_count_out':'move_count',
                                            'result_out':'action_result'})

        smach.StateMachine.add('COUNT', Count(),
                               transitions = {'count_success':'LOCALIZE',
                                              'count_failure':'FIND',
                                              'action_failed':'action_failed'},
                               remapping = {'target_name_in':'target_name',
                                            'sort_option_in':'sort_option',
                                            'e_l_count_in':'existence_loop_count',
                                            'sort_option_out':'sort_option',
                                            'bbox_out':'bbox'})

        smach.StateMachine.add('FIND', Find(),
                               transitions = {'find_success':'LOCALIZE',
                                              'find_failure':'MOVE'},
                               remapping = {'target_name_in':'target_name',
                                            'e_l_count_in':'existence_loop_count',
                                            'e_l_count_out':'existence_loop_count'})

        smach.StateMachine.add('LOCALIZE', Localize(),
                               transitions = {'localize_success':'CHECK_CENTER',
                                              'localize_failure':'MOVE'},
                               remapping = {'target_name_in':'target_name',
                                            'sort_option_in':'sort_option',
                                            'bbox_in':'bbox',
                                            'e_l_count_in':'existence_loop_count',
                                            'centroid_out':'centroid',
                                            'e_l_count_out':'existence_loop_count'})

        smach.StateMachine.add('CHECK_CENTER', CheckCenter(),
                               transitions = {'check_center_success':'success',
                                              'check_center_failure':'LOCALIZE',
                                              'action_failed':'action_failed'},
                               remapping = {'sort_option_in':'sort_option',
                                            'centroid_in':'centroid',
                                            'c_l_count_in':'center_loop_count',
                                            'sort_option_out':'sort_option',
                                            'c_l_count_out':'center_loop_count',
                                            'result_out':'action_result'})

        smach.StateMachine.add('MOVE', Move(),
                               transitions = {'retry':'COUNT'},
                               remapping = {'move_count_in':'move_count',
                                            'move_count_out':'move_count'})
         

　　　　"""クソ怪しいので書き直す可能性大

        asw = ActionServerWrapper('/recognition/action', RecognitionProcessingAction,
                                  wrapped_container = sm,
                                  succeeded_outcomes = ['success'],
                                  aborted_outcomes = ['action_failed'],
                                  preempted_outcomes = ['preempted'],
                                  goal_key='action_goal',
                                  feedback_key='action_feedback',
                                  result_key='action_result')
        asw.run_server()
　　　　
　　　　"""
rclpy.spin(node)
