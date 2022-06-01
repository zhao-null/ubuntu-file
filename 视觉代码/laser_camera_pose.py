#!/usr/bin/env python
#_*_ encoding:utf-8 _*_
from cmath import sqrt
import cv2 as cv
import jetson.inference
import jetson.utils

import rospy
import math
import time
import numpy as np
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
        self.poseArray = PoseArray()
        self.init_flag = 1
        #self.cv_image = []
        self.pose_edge = [0,640,0,480]
        self.last_pose_edge = []
        self.lasttime = time.time()
        #self.cv_image = np.array(self.cv_image)
        self.scanSubscriber = rospy.Subscriber('/scan', LaserScan, self.registerScan,queue_size=1)
        self.filterScanPublisher = rospy.Publisher('slave/filterScan', LaserScan,queue_size=1)
        self.objectScanPublisher = rospy.Publisher('slave/objectScan', LaserScan,queue_size=1)
        self.objectPublisher = rospy.Publisher('/objectPose', PoseArray,queue_size=1)
        #self.cameraSubscriber = rospy.Subscriber('/video_source/raw', Image, self.cameraImage, queue_size=1)
        #self.imagePublisher = rospy.Publisher("/usb_cam/detect_image", Image, queue_size=1)
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
        self.camera = jetson.utils.gstCamera(640,480,"/dev/video0")
        self.output = jetson.utils.videoOutput(self.opt.output_URI, argv=sys.argv)
        self.display = jetson.utils.glDisplay("display://0")

    def registerScan(self, scan_data):
        self.header = scan_data.header
        self.ranges = np.array(scan_data.ranges)
        self.angle_increment = scan_data.angle_increment
        self.angle_min = scan_data.angle_min
        self.filterScan = scan_data
        self.filterScan.ranges = [0] * self.ranges.size
        self.objectScan = scan_data
        self.objectScan.ranges = [0] * self.ranges.size
        self.cv_image,width,height = self.camera.CaptureRGBA()
        poses = self.net.Process(self.cv_image, overlay=self.opt.overlay)
        if(self.init_flag):
            max_width = 0
            for pose in poses:
                if(pose.Right-pose.Left > max_width):
                    self.pose_edge = [(int)(pose.Left),(int)(pose.Right),(int)(pose.Top),(int)(pose.Bottom)]
                    max_width = pose.Right-pose.Left
            if(self.pose_edge[0] is not 0 or self.pose_edge[0] is not 640):
                self.last_pose_edge = self.pose_edge
                self.init_flag = 0
        else:
            max_area = 0
            max_dis = 1000
            count = 0
            for pose in poses:
                left = max(self.last_pose_edge[0],int(pose.Left))
                right = min(self.last_pose_edge[1],int(pose.Right))
                top = max(self.last_pose_edge[2],int(pose.Top))
                bottom = max(self.last_pose_edge[3],int(pose.Bottom))
                center_x = (self.last_pose_edge[0] + self.last_pose_edge[1])/2 - (int(pose.Left) + int(pose.Right))/2
                center_y = (self.last_pose_edge[2] + self.last_pose_edge[3])/2 - (int(pose.Top) + int(pose.Bottom))/2
                dis = math.sqrt(center_x**2 + center_y**2)
                if(dis<max_dis):
                    index = count
                if((right-left)<0 or (bottom-top)<0):
                    area = 0
                else:
                    area = (right-left) * (bottom-top)
                if(area > max_area):
                    self.pose_edge = [(int)(pose.Left),(int)(pose.Right),(int)(pose.Top),(int)(pose.Bottom)]
                    max_area = area
                else:
                    pass
                count += 1
            if(max_area == 0):
                self.pose_edge = [(int)(poses[index].Left),(int)(pose[index].Right),(int)(pose[index].Top),(int)(pose[index].Bottom)]
            else:
                pass
            self.last_pose_edge = self.pose_edge

        if(not(self.init_flag)):
        #cv.rectangle(self.cv_image,((int)(self.pose_edge[0]),(int)(self.pose_edge[2])),((int)(self.pose_edge[1]),(int)(self.pose_edge[3])),(255,0,255),4)
            if(not(self.ranges is None) and not(self.cv_image is None)):
                flag = 1
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
                                if(flag):
                                    start = i
                                    flag = 0
                                end = i
                                self.filterScan.ranges[i] = self.ranges[i]
                                #cv.circle(self.cv_image, (x, y), 2, (0, 255, 255), thickness=2)
                            else:
                                pass
                                #cv.circle(self.cv_image, (x, y), 2, (255, 255, 0), thickness=-1)
                try:
                    self.pubulishPose(int(start+1),int(end-1),scan_data.angle_min,scan_data.angle_increment)
                except:
                    self.objectPublisher.publish(self.poseArray)
                    rospy.logwarn("there is no follow object,using the last pose!!!")
                self.filterScanPublisher.publish(self.filterScan)
                self.objectScanPublisher.publish(self.objectScan)
        else:
            rospy.logwarn("First person is finding,   initializing !!!")
            #self.display.RenderOnce(self.cv_image,width,height)
            #self.display.SetTitle ("Object Detection | Network {:.0f} FPS".format(self.net.GetNetworkFPS()))
            # self.display.RenderOnce(self.cv_image,width,height)
            # self.display.SetTitle ("Object Detection | Network {:.0f} FPS".format(self.net.GetNetworkFPS()))
            # self.output.Render(self.cv_image)
            # self.output.SetStatus("{:s} | Network {:.0f} FPS".format(self.opt.network, self.net.GetNetworkFPS()))
            #self.imagePublisher.publish(self.bridge.cv2_to_imgmsg(self.cv_image,'bgr8'))


    def pubulishPose(self,start,end,angle_min,angle_delta):
        pose = Pose()
        point_list = []
        temp = []
        set_list = []
        range_mid = 0
        angle_mid = 0
        count = 0
        for i in range(start,end+1):
            if(self.ranges[i] != float('inf') and self.ranges[i] != 0):
                if(self.ranges[i]<5.0):
                    point_list.append([self.ranges[i],i])
        for i in range(len(point_list)):
            if(i == len(point_list)-1):
                if(len(temp)<5):
                    temp[:]=[]
                else:
                    set_list.append(list(temp))
                    temp[:]=[]
            else:
                if(abs(point_list[i][0] - point_list[i+1][0])<0.1):
                    temp.append(point_list[i])
                else:
                    if(len(temp)<8):
                        temp[:]=[]
                    else:
                        set_list.append(list(temp))
                        temp[:]=[]
        for setx in set_list:
            for i in range(len(setx)):
                self.objectScan.ranges[setx[i][1]] = setx[i][0]
                range_mid += setx[i][0]
                angle_mid += setx[i][1]
                count += 1
        range_mid /= count
        angle_mid /= count
        angle_mid = angle_min + angle_mid*angle_delta
        pose.position.x = math.cos(angle_mid) * range_mid
        pose.position.y = math.sin(angle_mid) * range_mid
        pose.orientation.w = math.cos(angle_mid/2)
        pose.orientation.z = math.sin(angle_mid/2)
        self.poseArray.poses[:] = []
        self.poseArray.poses.append(pose)
        self.poseArray.header = self.header
        self.objectPublisher.publish(self.poseArray)

    # def cameraImage(self, data):
    #     try:
    #         self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    #         self.imagePublisher.publish(self.bridge.cv2_to_imgmsg(self.cv_image,'bgr8'))
    #         bgr_img = jetson.utils.cudaFromNumpy(self.cv_image,isBGR=True)
    #         img = jetson.utils.cudaAllocMapped(width=bgr_img.width,height=bgr_img.height,format='rgb8')
    #         jetson.utils.cudaConvertColor(bgr_img,img)
    #         poses = self.net.Process(img, overlay=self.opt.overlay)
    #         #print(type(img))
    #         print(time.time()-self.lasttime)
    #         self.lasttime = time.time()
    #         #print("detected {:d} objects in image".format(len(poses)))
    #         max_width = 0
    #         for pose in poses:
    #             if(pose.Right-pose.Left > max_width):
    #                 self.pose_edge = [(int)(pose.Left),(int)(pose.Right),(int)(pose.Top),(int)(pose.Bottom)]
    #                 max_width = pose.Right-pose.Left
    #         cv.rectangle(self.cv_image,(self.pose_edge[0],self.pose_edge[2]),(self.pose_edge[1],self.pose_edge[3]),(255,0,255),4)
    #         #self.imagePublisher.publish(self.bridge.cv2_to_imgmsg(self.cv_image,'bgr8'))
    #     except CvBridgeError as e:
    #         print(1)

if __name__ == '__main__':
	#time.sleep(1)
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

