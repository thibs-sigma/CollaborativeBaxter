#!/usr/bin/env python
import sys
import rospy
import roslaunch
import cv2
from std_msgs.msg import String, Bool, Float32, UInt32
from sensor_msgs.msg import Image
from baxter_core_msgs.msg import EndpointState, DigitalIOState, NavigatorState, DigitalOutputCommand
from cv_bridge import CvBridge

is_moving = False

img_waiting = cv2.imread('/home/thib/simulation_ws/src/launch_demo/msg/waiting.png')
msg_waiting = CvBridge().cv2_to_imgmsg(img_waiting, encoding="bgr8")

img_sleeping = cv2.imread('/home/thib/simulation_ws/src/launch_demo/msg/sleeping.png')
msg_sleeping = CvBridge().cv2_to_imgmsg(img_sleeping, encoding="bgr8")

img_assemblyTask = cv2.imread('/home/thib/simulation_ws/src/launch_demo/msg/assembly_task.png')
msg_assemblyTask = CvBridge().cv2_to_imgmsg(img_assemblyTask, encoding="bgr8")

img_inspectionTask = cv2.imread('/home/thib/simulation_ws/src/launch_demo/msg/inspection_task.png')
msg_inspectionTask = CvBridge().cv2_to_imgmsg(img_inspectionTask, encoding="bgr8")

img_pickupObject = cv2.imread('/home/thib/simulation_ws/src/launch_demo/msg/pickup_object.png')
msg_pickupObject = CvBridge().cv2_to_imgmsg(img_pickupObject, encoding="bgr8")

img_exit = cv2.imread('/home/thib/simulation_ws/src/launch_demo/msg/exit.png')
msg_exit = CvBridge().cv2_to_imgmsg(img_exit, encoding="bgr8")

def check_moving(data):
    global is_moving
    is_moving = data.data

# Callback OK wheel button left navigator (WORKING)
def navigatorCallback(data):
    global navigatorOK_state
    navigatorOK_state = data.buttons[0]
    # print (navigatorOK_state)

# Callback wheel index left navigator (WORKING)
def wheelIndexCallback(data):
    global wheelIndex_value
    wheelIndex_value = data.data
    # print (wheelIndex_value % 3)

def poll_object_request():
    while True:

        # image_pub.publish(msg_waiting)

        # Baxter's ready, all 'green'
        head_RedLed_pub.publish(0.0)
        head_GreenLed_pub.publish(100.0)

        # Light showing where user should interact with Baxter
        leftInnerLight_pub.publish('left_inner_light', True)

        # Wait
        rospy.sleep(0.25)
 
        # Debug terminal 
        # print wheelIndex_value % 3

        if (wheelIndex_value % 3) == 0 :
            # Debug terminal
            print "Pickup object"

            while navigatorOK_state != True:
                # BAXTER SCREEN OUTPUT
                image_pub.publish(msg_pickupObject)

            # Roslaunch stuff
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/thib/simulation_ws/src/object-recognition/launch/demo.launch"])
            launch.start()
            rospy.loginfo("Pickup object launchfile started")


                
        if (wheelIndex_value % 3) == 1 :
            # Debug terminal
            print "Assembly task"
            # desired_object = "enclosure"

            # BAXTER SCREEN OUTPUT
            image_pub.publish(msg_assemblyTask)
        
        if (wheelIndex_value % 3) == 2 :
            # Debug terminal
            print "EXIT!"
            desired_object = "q"
            # BAXTER SCREEN OUTPUT
            image_pub.publish(msg_exit)



        # desired_object = raw_input('Enter the object you would like to pick up (q to quit): ')


        if navigatorOK_state == True:
            if desired_object == 'q':
                print "EXIT requested"
                image_pub.publish(msg_sleeping)
                leftInnerLight_pub.publish('left_inner_light', False)
                rospy.sleep(1)
                break
            else:
                # Debug terminal
                # print "Finding and picking up ",desired_object
                # Switch off button light
                leftInnerLight_pub.publish('left_inner_light', False)
                desired_object_pub.publish(desired_object)
                print "PROGRAM requested"
                rospy.sleep(2)

        while is_moving:
            pass

    print "Done!"

if __name__ == '__main__':
    rospy.init_node('request_object', log_level=rospy.INFO)

    rate = rospy.Rate(100)
    desired_object_pub = rospy.Publisher("desired_object",String,queue_size=10)

    rospy.Subscriber("is_moving",Bool,check_moving)
    image_pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=10)

    head_RedLed_pub = rospy.Publisher('/robot/sonar/head_sonar/lights/set_red_level', Float32, queue_size=1)
    head_GreenLed_pub = rospy.Publisher('/robot/sonar/head_sonar/lights/set_green_level', Float32, queue_size=1)

    # Navigator subscribers
    rospy.Subscriber("/robot/navigators/left_navigator/state", NavigatorState, navigatorCallback) # OK Button
    rospy.Subscriber("/robot/analog_io/left_wheel/value_uint32", UInt32, wheelIndexCallback) # Wheel index

    leftInnerLight_pub = rospy.Publisher('/robot/digital_io/command', DigitalOutputCommand, queue_size=10)

    poll_object_request()
