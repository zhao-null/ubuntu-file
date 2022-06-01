# coding: utf-8
import platform
import rospy
import cv2
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import time


class photoSaver:
    def __init__(self):
        self.bridge = CvBridge()
        imageSubscriber = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")			# 获取订阅的摄像头图像
            cv2.imshow("capture", cv_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                # 存储图片
                cv2.imwrite("camera.jpg", cv_image)
        except CvBridgeError as e:
            print(e)

if __name__ == '__main__':
    try:
        rospy.init_node("cv_bridge_test")
        rospy.loginfo("Starting cv_bridge_test node")
        photoSaver()
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down cv_bridge_test node.")
        cv2.destroyAllWindows()




