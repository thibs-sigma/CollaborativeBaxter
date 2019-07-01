#!/usr/bin/env python
from grip_node import GripperClient

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import image_geometry
from std_msgs.msg import Bool, String, Float32, UInt32
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import Point
from baxter_core_msgs.msg import EndpointState, DigitalIOState, NavigatorState, DigitalOutputCommand
from object_recognition.msg import ObjectInfo
import planning_node_left as pnodeLeft
import planning_node_right as pnodeRight
import baxter_interface
import tf

# Initialization
camera_state_info = None
pixel_info = None
camera_info = None
object_location = None
object_orientation = None
object_name = None
desired_object = None
screwsDetected = 0

# screwingOK = False
# inspectionOK = False

img_gettingReady = cv2.imread('/home/thib/simulation_ws/src/inspection_task/msg/ready_inspection.png')
msg_gettingReady = CvBridge().cv2_to_imgmsg(img_gettingReady, encoding="bgr8")

img_inspectionRunning = cv2.imread('/home/thib/simulation_ws/src/inspection_task/msg/inspection_running.png')
msg_inspectionRunning = CvBridge().cv2_to_imgmsg(img_inspectionRunning, encoding="bgr8")

img_inspectionCompleted = cv2.imread('/home/thib/simulation_ws/src/inspection_task/msg/inspection_completed.png')
msg_inspectionCompleted = CvBridge().cv2_to_imgmsg(img_inspectionCompleted, encoding="bgr8")

img_movingScrewing = cv2.imread('/home/thib/simulation_ws/src/assembly_task/msg/moving_screwing.png')
msg_movingScrewing = CvBridge().cv2_to_imgmsg(img_movingScrewing, encoding="bgr8")

img_confirmScrewing = cv2.imread('/home/thib/simulation_ws/src/assembly_task/msg/confirm_screwing.png')
msg_confirmScrewing = CvBridge().cv2_to_imgmsg(img_confirmScrewing, encoding="bgr8")

img_screwingCompleted = cv2.imread('/home/thib/simulation_ws/src/assembly_task/msg/screwing_completed.png')
msg_screwingCompleted = CvBridge().cv2_to_imgmsg(img_screwingCompleted, encoding="bgr8")

img_missingScrews = cv2.imread('/home/thib/simulation_ws/src/inspection_task/msg/missing_screws.png')
msg_missingScrews = CvBridge().cv2_to_imgmsg(img_missingScrews, encoding="bgr8")

dinspection = [0.690, -0.018, 0.167, 0.353, 0.187, 0.781, -0.479]
dinspectionCamera = [0.666, -0.023, 0.188, 0.790, 0.464, -0.351, 0.191]
dplace = [0.880, -0.170, 0.221, 0.390, 0.118, 0.848, -0.338]
dhome = [0.705, -0.327, 0.125, 0.999, 0.016, 0.008, 0.010]
# Pre-recorded, can be adjusted by operator

# THIS WORKS


def initCamera(data):
    global camera_info
    camera_info = data

# THIS WORKS


def getCameraState(data):
    global camera_state_info
    camera_state_info = data

# THIS WORKS

# Callback OK wheel button left navigator (WORKING)
def navigatorCallback(data):
    global navigatorOK_state
    navigatorOK_state = data.buttons[0]
    # print (navigatorOK_state)

def screwsDetectedCallback(data):
    global screwsDetected
    screwsDetected = data.data
    # print (screwsDetected)


# Action when object picked up
def inspection():
    screwsDetectedMem = UInt32(0)
    
    gc = GripperClient()

    # BAXTER SCREEN OUTPUT
    # image_pub.publish(msg_confirm)
    
    rospy.logwarn_throttle(1,"Getting ready for inspection!")

    # Display action on Baxter's screen
    image_pub.publish(msg_gettingReady)

    # Wait
    rospy.sleep(1)

    pnodeRight.initplannode(dinspection, "right")

    # Wait
    rospy.sleep(1)

    pnodeLeft.initplannode(dinspectionCamera, "left")

    gc.command(position=0.0, effort=0.0) # Close gripper
    gc.wait()

    rospy.logwarn_throttle(1,"Inspection in progress...")
    


    # CHANGE THIS
    # while navigatorOK_state != True:    
    #     # BLINKING BUTTON LIGHT
    #     leftInnerLight_pub.publish('left_inner_light', True)
    #     rospy.sleep(0.5)
    #     leftInnerLight_pub.publish('left_inner_light', False)
    #     rospy.sleep(0.5)
    #     print screwsDetected

    while screwsDetected == 0:
        rospy.logwarn_throttle(1, "Inspection running")
        # Display action on Baxter's screen
        # image_pub.publish(msg_inspectionRunning)
    

    if screwsDetected == 4:
        #Debug terminal
        print "Inspection completed! 4/4 screws detected."
        
        # Wait
        rospy.sleep(2)
        # BAXTER SCREEN OUTPUT
        image_pub.publish(msg_inspectionCompleted)

        #Wait
        rospy.sleep(2)

        # Debug terminal
        print "Going back home"
        # Call function
        goingHome()

    else:
        print ("Missing " + str(4 - screwsDetected) + " screw(s)! Inspection to be done again...")
        
        # Wait
        rospy.sleep(2)
        # BAXTER SCREEN OUTPUT
        image_pub.publish(msg_missingScrews)

        #Wait
        rospy.sleep(2)

        assemblyAgain()
    


    return

# THIS WORKS

def goingHome():
    print ("Inside goingHome function")

    # Debug terminal
    print "Everything fine! Going home (and then placing bax the box on the table)"

    # Display action on Baxter's screen
    # image_pub.publish(msg_movingScrewing)   

    # Wait
    rospy.sleep(1)

    # Going home (better: going to PlaceBack script...)
    # pnodeRight.initplannode(dhome, "right")
    # Move the left arm as well (otherwise, collision)

    # Wait
    rospy.sleep(1)

    # Debug terminal
    print ("I'm back to home position")
    print ("Placing back the box... TO DO")

    return

def assemblyAgain():
    print ("Inside assemblyAgain function")
    
    # Debug terminal
    print "Presenting again the object to the operator..."

    # Display action on Baxter's screen
    image_pub.publish(msg_movingScrewing)

    # Wait
    rospy.sleep(1)

    # Present object to operator to be assembled
    pnodeRight.initplannode(dplace, "right")

    # Wait for screwing
    screwing()

    # Wait
    rospy.sleep(1)

    # Reset screwsDetected publisher
    screwsDetected_pub.publish(0)

    # Inspection again
    inspection()

    return

def screwing():
    
    # BAXTER SCREEN OUTPUT
    image_pub.publish(msg_confirmScrewing)
    
    rospy.logwarn_throttle(1,"Waiting for the action to be completed...")

    while navigatorOK_state != True:    
        # BLINKING BUTTON LIGHT
        leftInnerLight_pub.publish('left_inner_light', True)
        rospy.sleep(0.5)
        leftInnerLight_pub.publish('left_inner_light', False)
        rospy.sleep(0.5)
        
    print "You completed the action."
    
    # BAXTER SCREEN OUTPUT
    image_pub.publish(msg_screwingCompleted)


    return

def arm_setup():
    # Get desired joint values from parameter server
    left_w0 = rospy.get_param('left_w0', default=0)
    left_w1 = rospy.get_param('left_w1', default=0)
    left_w2 = rospy.get_param('left_w2', default=0)
    left_e0 = rospy.get_param('left_e0', default=0)
    left_e1 = rospy.get_param('left_e1', default=0)
    left_s0 = rospy.get_param('left_s0', default=0)
    left_s1 = rospy.get_param('left_s1', default=0)

    # Send the left arm to the desired position
    home = {'left_w0': left_w0, 'left_w1': left_w1, 'left_w2': left_w2,
            'left_e0': left_e0, 'left_e1': left_e1, 'left_s0': left_s0, 'left_s1': left_s1}
    limb = baxter_interface.Limb('left')
    limb.move_to_joint_positions(home)


if __name__ == '__main__':
    rospy.init_node('inspection', log_level=rospy.INFO)

    print "Moving arm to correct location"
    # arm_setup()

    

    # ROS stuff
    rospy.Subscriber("/cameras/left_hand_camera/camera_info", CameraInfo, initCamera)
    rospy.Subscriber("/robot/limb/left/endpoint_state", EndpointState, getCameraState)

    rate = rospy.Rate(50)
    while (camera_info is None) or (camera_state_info is None):
        rate.sleep()

    image_pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=10)
    is_moving_pub = rospy.Publisher("is_moving", Bool, queue_size=10)

    head_RedLed_pub = rospy.Publisher('/robot/sonar/head_sonar/lights/set_red_level', Float32, queue_size=1)
    head_GreenLed_pub = rospy.Publisher('/robot/sonar/head_sonar/lights/set_green_level', Float32, queue_size=1)

    leftInnerLight_pub = rospy.Publisher('/robot/digital_io/command', DigitalOutputCommand, queue_size=10)
    # leftOuterLight_pub = rospy.Publisher('/robot/digital_io/command', DigitalOutputCommand, queue_size=10)

    # object_location_pub = rospy.Publisher("object_location",ObjectInfo,queue_size=10)

    screwsDetected_pub = rospy.Publisher('/screwsDetected', UInt32, latch=True, queue_size=10) 

    # Buttons subscribers
    # rospy.Subscriber("/robot/digital_io/left_lower_button/state", DigitalIOState, buttonOKPress)
    rospy.Subscriber("/robot/navigators/left_navigator/state", NavigatorState, navigatorCallback)

    rospy.Subscriber("/screwsDetected", UInt32, screwsDetectedCallback)



    is_moving_pub.publish(False)


    # Debug terminal
    print "Ready to go!"

    inspection()

    rospy.spin()
