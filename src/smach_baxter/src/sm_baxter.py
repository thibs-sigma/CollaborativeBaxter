#!/usr/bin/env python
import os

from std_msgs.msg import String

import numpy as np
from numpy import linalg as LA

import rospy
import threading

import smach
from smach import State, StateMachine, Concurrence

import smach_ros
from smach_ros import ServiceState, SimpleActionState, MonitorState, IntrospectionServer

import std_srvs.srv

from geometry_msgs.msg import Twist

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose

from sensor_msgs.msg import JointState
from actionlib_msgs.msg import GoalStatus
from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import LinkStates
from math import sqrt

from moveit_msgs.msg import MoveItErrorCodes
moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name

# Import scripts
# import request_action


class RESET(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])
    
    def execute(self, userdata):
        print("Inside init state machine\n")
        print(rospy.get_name())

        # self.state = 0
        # rospy.sleep(1)
        # self.check_states()
  

        rospy.loginfo("State machine initialized!")
        return 'success'

class CHOOSEACTION(State):


    def __init__(self):
        State.__init__(self, outcomes=['success', 'stop'])
        
        # Subscribers
        rospy.Subscriber("/desired_object", String, self.actionRequestedCallback)

    def actionRequestedCallback(self, data):
        self.actionRequested_str = data.data
        # print (self.actionRequested_str)
        # return str(actionRequested_str)

    def execute(self, userdata):
        print("Inside CHOOSEACTION state machine\n")
        pwd = os.getcwd()
        os.system(pwd + "/src/launch_demo/src/request_action.py")
        # rospy.loginfo("State machine initialized!")
        print(self.actionRequested_str)
        rospy.sleep(1)
        if self.actionRequested_str == 'q':
            return 'stop'
        elif self.actionRequested_str == 'assembly':
            print ('callback assembly, stateMachine to be completed')
            return 'success'

class PICKUPOBJECT(State):


    def __init__(self):
        State.__init__(self, outcomes=['success', 'fail'])
        
        # Subscribers
        rospy.Subscriber("/desired_object", String, self.actionRequestedCallback)

    def actionRequestedCallback(self, data):
        self.actionRequested_str = data.data
        # print (self.actionRequested_str)
        # return str(actionRequested_str)

    def execute(self, userdata):
        print("Inside PICKUPOBJECT state machine\n")
        pwd = os.getcwd()
        os.system(pwd + "/src/launch_demo/src/request_action.py")
        # rospy.loginfo("State machine initialized!")
        print(self.actionRequested_str)
        rospy.sleep(1)
        if self.actionRequested_str == 'q':
            return 'success'
        elif self.actionRequested_str == 'assembly':
            print ('callback assembly, stateMachine to be completed')
            return 'fail'


if __name__ == "__main__":

    try:
        rospy.init_node('baxter_SMACH')
        

        # Create a SMACH state machine
        sm = StateMachine(outcomes=['success', 'fail', 'stop'])

        # Open the container (HERE IS DEFINED THE SM)
        with sm:
            # Reset the Baxter
            StateMachine.add('RESET', RESET(), transitions={'success':'CHOOSE_ACTION'})
            StateMachine.add('CHOOSE_ACTION', CHOOSEACTION(), transitions={'success':'PICKUP_OBJECT', 'stop':'stop'})
            StateMachine.add('PICKUP_OBJECT', PICKUPOBJECT(), transitions={'success':'success', 'fail':'fail'})
        
        # Attach a SMACH introspection server
        sis = IntrospectionServer('baxter_SMACH_introspection', sm, '/BAXTER_DEMO')
        sis.start()

        # Set preempt handler
        smach_ros.set_preempt_handler(sm)

        # Execute SMACH tree in a separate thread so that we can ctrl-c the script
        smach_thread = threading.Thread(target = sm.execute)
        smach_thread.start()


    
    except rospy.ROSInterruptException:
        pass


    # Signal handler (wait for CTRL+C)
    rospy.spin()
