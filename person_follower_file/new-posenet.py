#!/usr/bin/env python
#_*_ encoding:utf-8 _*_
import jetson.inference
import jetson.utils

import rospy
import math
import time
import numpy as np
import cv2 as cv
from visualization_msgs.msg import Marker,MarkerArray
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import  LaserScan,Image
from geometry_msgs.msg import Pose,PoseArray,Point,Pose2D

import argparse
import sys



class personFollower:
    def __init__(self):
        self.a = [-6.69249150, 4.89812674, 395.065579, -5.34907425, -0.201485137, 602.245746, -0.0186462658, -0.000573126502, 1]
        self.bridge = CvBridge()
        self.cv_image = []
        self.pose_edge = []
        self.lasttime = time.time()
        self.cv_image = np.array(self.cv_image)
        self.scanSubscriber = rospy.Subscriber('/scan', LaserScan, self.registerScan)
        self.filterScanPublisher = rospy.Publisher('slave/filterScan', LaserScan,queue_size=1000)
        self.cameraSubscriber = rospy.Subscriber('/usb_cam/image_raw', Image, self.cameraImage, queue_size=1)
        self.imagePublisher = rospy.Publisher("/usb_cam/detect_image", Image, queue_size=1)
        # parse the command line
        parser = argparse.ArgumentParser(description="Run pose estimation DNN on a video/image stream.", 
                                        formatter_class=argparse.RawTextHelpFormatter, epilog=jetson.inference.poseNet.Usage() +
                                        jetson.utils.videoSource.Usage() + jetson.utils.videoOutput.Usage() + jetson.utils.logUsage())

        parser.add_argument("input_URI", type=str, default="", nargs='?', help="URI of the input stream")
        parser.add_argument("output_URI", type=str, default="", nargs='?', help="URI of the output stream")
        parser.add_argument("--network", type=str, default="resnet18-body", help="pre-trained model to load (see below for options)")
        parser.add_argument("--overlay", type=str, default="links,keypoints", help="pose overlay flags (e.g. --overlay=links,keypoints)\nvalid combinations are:  'links', 'keypoints', 'boxes', 'none'")
        parser.add_argument("--threshold", type=float, default=0.15, help="minimum detection threshold to use") 

        try:
            self.opt = parser.parse_known_args()[0]
        except:
            print("")
            parser.print_help()
            sys.exit(0)

        # load the pose estimation model
        self.net = jetson.inference.poseNet(self.opt.network, sys.argv, self.opt.threshold)

    def registerScan(self, scan_data):
        self.header = scan_data.header
        self.ranges = np.array(scan_data.ranges)
        self.angle_increment = scan_data.angle_increment
        self.angle_min = scan_data.angle_min
        self.filterScan = scan_data
        self.filterScan.ranges = [0] * self.ranges.size
        if(not(self.ranges is None) and not(self.cv_image is None)):
            for i in range(1,self.ranges.size-1):
                if(self.ranges[i] != float('inf') and self.ranges[i]!=0):
                    angle = self.angle_increment * i + self.angle_min
                    if (angle>-45*math.pi/180 and angle<45*math.pi/180):
                        u = (int)(1000*self.ranges[i]*math.cos(angle))
                        v = (int)(1000*self.ranges[i]*math.sin(angle))
                        lamb = u*self.a[6] + v*self.a[7] + self.a[8]
                        x = int((u*self.a[0] + v*self.a[1] + self.a[2]) / lamb)
                        y = int((u*self.a[3] + v*self.a[4] + self.a[5]) / lamb)
                        if(x > self.pose_edge[0] and x < self.pose_edge[1]):
                            self.filterScan.ranges[i] = self.ranges[i]
                            cv.circle(self.cv_image, (x, y), 2, (0, 255, 255), thickness=2)
                        else:
                            cv.circle(self.cv_image, (x, y), 2, (255, 255, 0), thickness=-1)
        self.filterScanPublisher.publish(self.filterScan)
        self.imagePublisher.publish(self.bridge.cv2_to_imgmsg(self.cv_image,'bgr8'))

    def cameraImage(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "rgb8")
            bgr_img = jetson.utils.cudaFromNumpy(self.cv_image,isRGB=True)
            img = jetson.utils.cudaAllocMapped(width=bgr_img.width,height=bgr_img.height,format='rgb8')
            jetson.utils.cudaConvertColor(bgr_img,img)
            poses = self.net.Process(img, overlay=self.opt.overlay)
            #print(type(img))
            #print(time.time()-self.lasttime)
            self.lasttime = time.time()
            #print("detected {:d} objects in image".format(len(poses)))
            max_width = 0
            for pose in poses:
                if(pose.Right-pose.Left > max_width):
                    self.pose_edge = [(int)(pose.Left),(int)(pose.Right),(int)(pose.Top),(int)(pose.Bottom)]
                    max_width = pose.Right-pose.Left
            cv.rectangle(self.cv_image,(self.pose_edge[0],self.pose_edge[2]),(self.pose_edge[1],self.pose_edge[3]),(255,0,255),4)
            #self.imagePublisher.publish(self.bridge.cv2_to_imgmsg(self.cv_image,'bgr8'))
        except CvBridgeError as e:
            print(e)

if __name__ == '__main__':
	time.sleep(1)
	try:
		rospy.init_node("laser_on_camera")
		rospy.loginfo("Starting laser_on_camera node")
		personFollower()
		rospy.spin()
		if(rospy.is_shutdown()):
			cv.destroyAllWindows()
			print ("Shutting down laser_on_camera node.")
	except KeyboardInterrupt:
		print ("Shutting down laser_on_camera node.")
		cv.destroyAllWindows()

