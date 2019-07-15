#!/usr/bin/env python
import os
import subprocess

from std_msgs.msg import String, Bool

from cv_bridge import CvBridge
from cv2 import imread
from sensor_msgs.msg import Image

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

from object_recognition.msg import ObjectInfo

from moveit_msgs.msg import MoveItErrorCodes
moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name

# Import scripts
# import request_action

img_untuckingArms = imread('/home/ridgebackbaxter/CollaborativeBaxter_ws/src/launch_demo/msg/untucking_arms.png')
msg_untuckingArms = CvBridge().cv2_to_imgmsg(img_untuckingArms, encoding="bgr8")

img_tuckingArms = imread('/home/ridgebackbaxter/CollaborativeBaxter_ws/src/launch_demo/msg/tucking_arms.png')
msg_tuckingArms = CvBridge().cv2_to_imgmsg(img_tuckingArms, encoding="bgr8")

img_readyAssembly = imread('/home/ridgebackbaxter/CollaborativeBaxter_ws/src/launch_demo/msg/ready_assembly_task.png')
msg_readyAssembly = CvBridge().cv2_to_imgmsg(img_readyAssembly, encoding="bgr8")

img_readyPickup = imread('/home/ridgebackbaxter/CollaborativeBaxter_ws/src/launch_demo/msg/ready_pickup_object.png')
msg_readyPickup = CvBridge().cv2_to_imgmsg(img_readyPickup, encoding="bgr8")


class RESET(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])
    
    def execute(self, userdata):
        print("Inside init state machine\n")
        print(rospy.get_name())

        # Wait
        rospy.sleep(2)

        rospy.loginfo("State machine initialized!")

        return 'success'

class UNTUCK(State):
    def __init__(self):
        State.__init__(self, outcomes=['success', 'stop'])
        
    def execute(self, userdata):
        print("Inside UNTUCK state machine\n")
        # pwd = os.getcwd()
        # os.system(pwd + "/src/baxter/baxter_tools/scripts/tuck_arms.py -u")

        # Baxter screen output
        image_pub.publish(msg_untuckingArms)
        
        # Wait
        rospy.sleep(1)
        
        # Call 'untuck_arms' script
        # os.system("/home/ridgebackbaxter/CollaborativeBaxter_ws/src/baxter/baxter_tools/scripts/tuck_arms.py -u")
        subprocess.check_call(["/home/ridgebackbaxter/CollaborativeBaxter_ws/src/baxter/baxter_tools/scripts/tuck_arms.py", "-u"])

        # Wait for termination
        rospy.sleep(1)
        
        return 'success'

class TUCK(State):
    def __init__(self):
        State.__init__(self, outcomes=['success', 'stop'])
        
    def execute(self, userdata):
        print("Inside TUCK state machine\n")
        # pwd = os.getcwd()
        # os.system(pwd + "/src/baxter/baxter_tools/scripts/tuck_arms.py -t")

        # Baxter screen output
        image_pub.publish(msg_tuckingArms)

        # Wait
        rospy.sleep(1)

        # Call 'tuck_arms' script
        # os.system("/home/ridgebackbaxter/CollaborativeBaxter_ws/src/baxter/baxter_tools/scripts/tuck_arms.py -t")
        subprocess.check_call(["/home/ridgebackbaxter/CollaborativeBaxter_ws/src/baxter/baxter_tools/scripts/tuck_arms.py", "-t"])

        # Wait for termination
        rospy.sleep(1)

        return 'success'


class CHOOSEACTION(State):
    def __init__(self):
        State.__init__(self, outcomes=['assemblyRequest', 'pickupRequest', 'tuckRequest', 'untuckRequest', 'stop'])
        
        # Subscribers
        rospy.Subscriber("/desired_object", String, self.actionRequestedCallback)

    def actionRequestedCallback(self, data):
        self.actionRequested_str = data.data
        # print (self.actionRequested_str)
        # return str(actionRequested_str)

    def execute(self, userdata):
        print("Inside CHOOSEACTION state machine\n")
        # pwd = os.getcwd()
        # os.system(pwd + "/src/launch_demo/src/request_action.py")
        
        # Call 'request_action' script
        # os.system("/home/ridgebackbaxter/CollaborativeBaxter_ws/src/launch_demo/src/request_action.py")
        subprocess.check_call("/home/ridgebackbaxter/CollaborativeBaxter_ws/src/launch_demo/src/request_action.py") # Better way to call external python script
        
        # Wait for termination
        rospy.sleep(1)

        if self.actionRequested_str == 'q':
            print ('Stop requested')
            return 'stop'
        elif self.actionRequested_str == 'assembly':
            print ('Assembly requested')
            return 'assemblyRequest'
        elif self.actionRequested_str == 'pickup':
            print ('Pickup requested')
            return 'pickupRequest'
        elif self.actionRequested_str == 'tuck':
            print ('Tuck requested')
            return 'tuckRequest'
        elif self.actionRequested_str == 'untuck':
            print ('Untuck requested')
            return 'untuckRequest'


# ASSEMBLY TASK
class MENUASSEMBLY(State):
    def __init__(self):
        State.__init__(self, outcomes=['requestOK', 'stop'])
        
        # Subscribers
        rospy.Subscriber("/desired_object", String, self.actionRequestedCallback)
        
    def actionRequestedCallback(self, data):
        self.actionRequested_str = data.data
        # print (self.actionRequested_str)
        # return str(actionRequested_str)

    def execute(self, userdata):
        print("Inside MENUASSEMBLY state machine\n")
        # pwd = os.getcwd()
        # os.system(pwd + "/src/launch_demo/src/request_action.py")
        # os.system("/home/ridgebackbaxter/CollaborativeBaxter_ws/src/smach_baxter/src/request_object_SMACH.py")

        # Baxter screen output
        image_pub.publish(msg_readyAssembly)
        
        # Wait
        rospy.sleep(1)
        
        # Call 'request_object' script
        # os.system("/home/ridgebackbaxter/CollaborativeBaxter_ws/src/assembly_task/src/request_object.py")
        subprocess.check_call("/home/ridgebackbaxter/CollaborativeBaxter_ws/src/assembly_task/src/request_object.py")

        # Wait for termination
        rospy.sleep(1)

        if self.actionRequested_str == 'q':
            return 'stop'
        elif self.actionRequested_str == 'enclosure':
            return 'requestOK'

class OBJECTPREDICTION(State):
    def __init__(self):
        State.__init__(self, outcomes=['predictionOK', 'stop'])
        
    def execute(self, userdata):
        print("Inside OBJECTPREDICTION state machine\n")
        # pwd = os.getcwd()
        # os.system("/home/ridgebackbaxter/CollaborativeBaxter_ws/src/assembly_task/src/baxter_object_prediction.py")

        # Call 'baxter_object_prediction' script
        subprocess.check_call("/home/ridgebackbaxter/CollaborativeBaxter_ws/src/assembly_task/src/baxter_object_prediction.py")

        # Wait for termination
        rospy.sleep(1)

        return 'predictionOK'

class ASSEMBLY(State):
    def __init__(self):
        State.__init__(self, outcomes=['success', 'stop'])

    def actionRequestedCallback(self, data):
        self.actionRequested_str = data.data
        # print (self.actionRequested_str)
        # return str(actionRequested_str)

    def execute(self, userdata):
        print("Inside ASSEMBLY state machine\n")
        # pwd = os.getcwd()
        # os.system("/home/ridgebackbaxter/CollaborativeBaxter_ws/src/assembly_task/src/pickup_box.py")

        # Call 'pickup_box' script
        subprocess.check_call("/home/ridgebackbaxter/CollaborativeBaxter_ws/src/assembly_task/src/pickup_box.py")

        # Wait for termination
        rospy.sleep(1)

        return 'success'

# PICKUP TASK
class MENUPICKUP(State):
    def __init__(self):
        State.__init__(self, outcomes=['requestOK', 'stop'])
        
        # Subscribers
        rospy.Subscriber("/desired_object", String, self.actionRequestedCallback)        

    def actionRequestedCallback(self, data):
        self.actionRequested_str = data.data
        # print (self.actionRequested_str)

    def execute(self, userdata):
        print("Inside MENUPICKUP state machine\n")
        # pwd = os.getcwd()
        # os.system(pwd + "/src/launch_demo/src/request_action.py")
        # os.system("/home/ridgebackbaxter/CollaborativeBaxter_ws/src/smach_baxter/src/request_object_SMACH.py")

        # Baxter screen output
        image_pub.publish(msg_readyPickup)

        # Wait
        rospy.sleep(1)

        # Call 'request_object' script
        # os.system("/home/ridgebackbaxter/CollaborativeBaxter_ws/src/object-recognition/src/baxter_demo/request_object.py")
        subprocess.check_call("/home/ridgebackbaxter/CollaborativeBaxter_ws/src/object-recognition/src/baxter_demo/request_object.py")

        # Wait for termination
        rospy.sleep(1)

        if self.actionRequested_str == 'q':
            return 'stop'
        elif self.actionRequested_str is not None:
            return 'requestOK'

class PICKUPOBJECTPREDICTION(State):
    def __init__(self):
        State.__init__(self, outcomes=['predictionOK', 'stop'])
    
    def execute(self, userdata):
        print("Inside OBJECTPREDICTION state machine\n")

        # pwd = os.getcwd()

        # Call 'baxter_object_prediction' script
        # os.system("/home/ridgebackbaxter/CollaborativeBaxter_ws/src/object-recognition/src/baxter_demo/baxter_object_prediction.py")
        subprocess.check_call("/home/ridgebackbaxter/CollaborativeBaxter_ws/src/object-recognition/src/baxter_demo/baxter_object_prediction.py")

        # Wait for termination
        rospy.sleep(1)

        return 'predictionOK'

class PICKUP(State):
    def __init__(self):
        State.__init__(self, outcomes=['success', 'stop'])
        
    def execute(self, userdata):
        print("Inside PICKUP state machine\n")

        # Call 'pickup_object' script
        # pwd = os.getcwd()
        # os.system(pwd + "/src/assembly_task/src/request_action.py")
        subprocess.check_call("/home/ridgebackbaxter/CollaborativeBaxter_ws/src/object-recognition/src/baxter_demo/pickup_object.py")

        # Wait for termination
        rospy.sleep(1)

        return 'success'

# INSPECTION TASK
class INSPECTIONPREDICTION(State):
    def __init__(self):
        State.__init__(self, outcomes=['predictionOK', 'stop'])
        
    def execute(self, userdata):
        print("Inside INSPECTIONPREDICTION state machine\n")

        # Call 'baxter_screws_prediction' script
        # pwd = os.getcwd()
        # os.system(pwd + "/src/assembly_task/src/request_action.py")
        subprocess.check_call("/home/ridgebackbaxter/CollaborativeBaxter_ws/src/inspection_task/src/baxter_screws_prediction.py")

        # Wait for termination
        rospy.sleep(1)

        return 'predictionOK'

class INSPECTION(State):
    def __init__(self):
        State.__init__(self, outcomes=['success', 'stop'])
        
    def execute(self, userdata):
        print("Inside INSPECTION state machine\n")
        
        # Call 'inspection' script
        # pwd = os.getcwd()
        subprocess.check_call("/home/ridgebackbaxter/CollaborativeBaxter_ws/src/inspection_task/src/inspection.py")

        # Wait for termination
        rospy.sleep(1)

        return 'success'

if __name__ == "__main__":
    image_pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=10)

    try:
        rospy.init_node('baxter_SMACH')
        
        # Create a SMACH state machine
        sm = StateMachine(outcomes=['success', 'fail', 'stop'])

        # Open the container (HERE IS DEFINED THE SM)
        with sm:
            # Reset the Baxter
            StateMachine.add('RESET', RESET(), transitions={'success':'UNTUCK'})
            StateMachine.add('CHOOSE_ACTION', CHOOSEACTION(), transitions={'assemblyRequest':'ASSEMBLY_ACTION', 'pickupRequest':'PICKUP_ACTION', 'tuckRequest':'TUCK', 'untuckRequest':'UNTUCK', 'stop':'stop'})
            StateMachine.add('TUCK', TUCK(), transitions={'success':'CHOOSE_ACTION', 'stop':'stop'})
            StateMachine.add('UNTUCK', UNTUCK(), transitions={'success':'CHOOSE_ACTION', 'stop':'stop'})

            # Concurrence allow to execute two parallel states

            # PICKUP TASK
            sm_pickup = Concurrence(outcomes=['success', 'fail'], default_outcome='fail', outcome_map={'success':{'PICKUPOBJECT_PREDICTION':'predictionOK', 'PICKUP':'success', 'MENU_PICKUP':'requestOK'}, 'fail':{'PICKUPOBJECT_PREDICTION':'stop', 'PICKUP':'stop', 'MENU_PICKUP':'stop'}})

            with sm_pickup:
                Concurrence.add('MENU_PICKUP', MENUPICKUP())
                Concurrence.add('PICKUPOBJECT_PREDICTION', PICKUPOBJECTPREDICTION())
                Concurrence.add('PICKUP', PICKUP())
                
            StateMachine.add('PICKUP_ACTION', sm_pickup, transitions={'success':'CHOOSE_ACTION', 'fail':'stop'})  


            # ASSEMBLY TASK
            sm_assembly = Concurrence(outcomes=['success', 'fail'], default_outcome='fail', outcome_map={'success':{'OBJECT_PREDICTION':'predictionOK', 'ASSEMBLY':'success', 'MENU_ASSEMBLY':'requestOK'}})

            with sm_assembly:
                Concurrence.add('MENU_ASSEMBLY', MENUASSEMBLY())
                Concurrence.add('OBJECT_PREDICTION', OBJECTPREDICTION())
                Concurrence.add('ASSEMBLY', ASSEMBLY())
                
            StateMachine.add('ASSEMBLY_ACTION', sm_assembly, transitions={'success':'INSPECTION_ACTION', 'fail':'stop'})

   

            # INSPECTION TASK
            sm_inspection = Concurrence(outcomes=['success', 'fail'], default_outcome='fail', outcome_map={'success':{'INSPECTION_PREDICTION':'predictionOK', 'INSPECTION':'success'}})

            with sm_inspection:
                Concurrence.add('INSPECTION_PREDICTION', INSPECTIONPREDICTION())
                Concurrence.add('INSPECTION', INSPECTION())
                
            StateMachine.add('INSPECTION_ACTION', sm_inspection, transitions={'success':'CHOOSE_ACTION', 'fail':'stop'})           
        
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



