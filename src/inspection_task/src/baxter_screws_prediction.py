#!/usr/bin/env python
import time
import sys
import rospy
import cv2
import numpy as np
import os
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sklearn.externals import joblib
from scipy.cluster.vq import vq, kmeans, whiten
from object_recognition.msg import ObjectInfo

class webcam_image:
    def __init__(self):
        self.rate = rospy.Rate(10)
        self.bridge = CvBridge()
        # baxter camera Subscriber
        self.image_sub = rospy.Subscriber("/cameras/left_hand_camera/image",Image,self.callback)
        # Subscriber to determine whether or not to use vision
        self.is_moving_sub = rospy.Subscriber("is_moving",Bool,self.check_moving)
        # Publisher
        # self.object_location_pub = rospy.Publisher("object_location",ObjectInfo,queue_size=10)


        # self.object_info = ObjectInfo()
        # self.object_info.names = ['','','']
        # self.object_info.x = [0,0,0]
        # self.object_info.y = [0,0,0]
        # self.object_info.theta = [0,0,0]
        self.is_moving = False

        self.directory = '/home/thib/simulation_ws/src/inspection_task/src'
        print "DIRECTORY IS: ",self.directory
        if not os.path.exists(self.directory):
            os.mkdir(self.directory)


    def check_moving(self,data):
        self.is_moving = data.data

    def callback(self,data):
        if not self.is_moving:
            try:
                frame = self.bridge.imgmsg_to_cv2(data,"bgr8")
            except CvBridgeError as e:
                print("==[CAMERA MANAGER]==",e)

            scale = 0.75
            frame = (frame*scale).astype(np.uint8)

            crop_img = frame[57:368, 85:584] # Use numpy slicing to crop the input image (no more background issues)
            cv2.imshow('crop img (video)',crop_img) 

            if cv2.waitKey(1) & 0xFF == ord('c'):
                image_path = self.directory + '/' + str("capture_inspection") + '.png'
                cv2.imwrite(image_path, crop_img)
                print "Capturing image",image_path


                self.screwDetection(crop_img)
                



            # self.screwDetection(crop_img)
            
            cv2.imshow("Camera input (video)",frame)

            # self.screwDetection(frame)
            cv2.waitKey(3)

            # Split frame into left and right halves
            # left_frame = frame[0:frame.shape[0], 0:frame.shape[1]/3]
            # middle_frame = frame[0:frame.shape[0], frame.shape[1]/3:2*frame.shape[1]/3]
            # right_frame = frame[0:frame.shape[0], 2*frame.shape[1]/3:frame.shape[1]]

            # Create list of left and right images
            # frames = [left_frame, middle_frame, right_frame]

    def screwDetection(self, frame):

            # for i,f in enumerate(frames):
            kps, des = sift.detectAndCompute(frame, None)
            # frames[i] = cv2.drawKeypoints(f, kps, None, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS) # draw SIFT points
            cv2.drawKeypoints(frame, kps, None, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS) # draw SIFT points
            

            # Convert current index to one_hot list representation for color assignment later
            # one_hot = [0,0,0]
            # one_hot[i] = 1

            # Basic threshold example
            gray = cv2.cvtColor(frame, cv2.COLOR_RGBA2GRAY)

            # mask = cv2.inRange(gray, 106, 255)

            # CHANGE THESE VALUES TO CALIBRATE BOUNDING BOXES
            th, dst = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY)
            # dst = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 19, 12)

            # new_img = cv2.bitwise_not(dst, mask)
            cv2.imshow('frame',frame)
            # cv2.imshow('mask',mask)
            cv2.imshow('res',dst)

            # crop_img_gray = dst[57:368, 85:584] # Use numpy slicing to crop the input image (no more background issues)
            # crop_img = frame[57:368, 85:584] # Use numpy slicing to crop the input image (no more background issues)

            # cv2.imshow('crop img',crop_img_gray) 

            # Find contours on binary image (cropped)
            img_contours, contours, hierarchy = cv2.findContours(dst,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

            contour_area=0
            areamax=0
            cxmax=0
            cymax=0  
            cntmax= None

            # Test Contour detection (output OK, but not counting properly...)
            # if len(contours) > 0:
            #     i = 0
            #     while contours:
            #         i = (i+1)
            #         contours = contours.next()
            #     print "Nb contours", len(contours)
            #     # cnt=contours[len(contours)-1]
            #     # cv2.drawContours(img_contours2, [cnt], 0, (0,255,0), 3)

            #     for cnt in contours:
            #         moments = cv2.moments(cnt)                          # Calculate moments
            #         contour_area = moments['m00']                    	# Contour area from moment
            #         # print "moments['m00']: ", moments['m00']
            #         if moments['m00']>750:
            #             cx = int(moments['m10']/moments['m00'])         # cx = M10/M00
            #             cy = int(moments['m01']/moments['m00'])         # cy = M01/M00 
            #             cxmax=cx
            #             cymax=cy
            #             cntmax= cnt
            #             areamax= contour_area
                        
            #         if cntmax is not None:
            #             (x,y),radius = cv2.minEnclosingCircle(cntmax)
            #             radius = int(radius)
            #             print "radius: ", radius
            #             # cv2.drawContours(frame,[cntmax],0,(0,255,0),1)   # draw contours in green color
            #             cv2.circle(frame,(cxmax,cymax),radius,(0,255,0),12) # Draw plain circle   
            #             cv2.imshow('output',frame)
            # else:
            #     print "No contour found..."

            # detectionGray = cv2.cvtColor(frame, cv2.COLOR_RGBA2GRAY)
            # cv2.imshow('detectionGray (count detected screws)', gray)

            # Test Hough Circles Detection (this is working, check robustness)
            gray = cv2.medianBlur(gray, 5)
            cv2.imshow('gray after blur', gray)
            nbScrews = cv2.HoughCircles(gray,cv2.HOUGH_GRADIENT,1,20,param1=100,param2=15,minRadius=18,maxRadius=23)
            if nbScrews is not None:
                nbScrews = np.uint16(np.around(nbScrews))
                for i in nbScrews[0,:]:
                    # draw the outer circle
                    cv2.circle(frame,(i[0],i[1]),i[2],(0,255,0),2)
                    # draw the center of the circle
                    cv2.circle(frame,(i[0],i[1]),2,(0,0,255),3)
                print "echo (nbscrews): ", nbScrews
                print "Nb screws: ", nbScrews.shape[1]
                # cv2.circle(gray,(nbScrews[0],nbScrews[1]),nbScrews[2],(0,255,0),12) # Draw plain circle
                cv2.imshow('detectionHough (count detected screws)', frame)
            else:
                print "No screw detected!"
                        

            
            cv2.waitKey(1)



def main(args):
    rospy.init_node('webcam_image', anonymous=True)
    ic = webcam_image()

    

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    print "OpenCV Version:",cv2.__version__

    # Load the classifier, class names, scaler, number of clusters and vocabulary
    classifier, class_names, std_slr, k, vocabulary = joblib.load("/home/thib/simulation_ws/src/assembly_task/src/dataset_RPLv2.pkl")

    # Create SIFT object
    sift = cv2.xfeatures2d.SIFT_create()

    predictions = [['','','','',''],['','','','',''],['','','','','']]
    counter = 0
    needResizing = False

    main(sys.argv)
