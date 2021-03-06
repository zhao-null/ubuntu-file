/*
 * @Copyright (c) 2017 DJI
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <iostream>
#include "tracking_utility.hpp"
//ROS
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <cv_bridge/cv_bridge.h>
#include "kcftracker.hpp"

#include <cstdio>
#include <chrono>

typedef std::chrono::time_point<std::chrono::high_resolution_clock> timer;
typedef std::chrono::duration<float> duration;

//global image data
cv::Mat  frame;


//using namespace DJI::OSDK;
using namespace std;
using namespace cv;


void imageCallBack(const sensor_msgs::ImageConstPtr& msg)
{

  ROS_DEBUG("H20T image received.");

  cv_bridge::CvImageConstPtr cam_image;

  try {
    cam_image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    frame = cam_image->image;
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc,argv,"KCFTracker");
  ros::NodeHandle nh("~");
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber imageSubscriber = it.subscribe("/usb_cam/image_raw",10,imageCallBack);

  // Initialize variables
  int functionTimeout = 1;

  //CameraRGBImage mainImg;
  const char winName[]="My Camera";
  char message1[100];
  char message2[100];
  Rect roi(0,0,0,0);

  KCFTracker *tracker = NULL;
  TrackingUtility tu;

  cv::namedWindow(winName,1);
  cv::setMouseCallback(winName,TrackingUtility::mouseCallback, (void*)&tu);

  while(1)
  {
    ros::spinOnce();
    char c = cv::waitKey(10);
    if(c==27)
    {
      if(tracker != NULL)
      {
        delete tracker;
        tracker = NULL;
      }
      break; // Quit if ESC is pressed
    }

    tu.getKey(c); //Internal states will be updated based on key pressed.

    if(!frame.empty())//when new image is ready.
    {
      int dx = 0;
      int dy = 0;
      timer trackerStartTime, trackerFinishTime;
      duration trackerTimeDiff;

      switch(tu.getState())
      {
      case TrackingUtility::STATE_IDLE:
        roi = tu.getROI();
        sprintf(message2, "Please select ROI and press g");
        break;

      case TrackingUtility::STATE_INIT:
        cout << "g pressed, initialize tracker" << endl;
        sprintf(message2, "g pressed, initialize tracker");
        roi = tu.getROI();
        tracker = new KCFTracker(true, true, false, false);
        tracker->init(roi, frame);
        tu.startTracker();
        break;

      case TrackingUtility::STATE_ONGOING:
        trackerStartTime  = std::chrono::high_resolution_clock::now();
        roi = tracker->update(frame);
        trackerFinishTime = std::chrono::high_resolution_clock::now();
        trackerTimeDiff = trackerFinishTime - trackerStartTime;
        sprintf(message2, "Tracking: bounding box update time = %.2f ms\n", trackerTimeDiff.count()*1000.0);
        break;

      case TrackingUtility::STATE_STOP:
        cout << "s pressed, stop tracker" << endl;
        sprintf(message2, "s pressed, stop tracker");
        delete tracker;
        tracker = NULL;
        tu.stopTracker();
        break;

      default:
        break;
      }

      dx = roi.x + roi.width/2  - frame.cols/2;
      dy = roi.y + roi.height/2 - frame.rows/2;

      cv::circle(frame, Point(frame.cols/2, frame.rows/2), 5, cv::Scalar(255,0,0), 2, 8);
      if(roi.width != 0)
      {
        cv::circle(frame, Point(roi.x + roi.width/2, roi.y + roi.height/2), 3, cv::Scalar(0,0,255), 1, 8);

        cv::line(frame,  Point(frame.cols/2, frame.rows/2),
                 Point(roi.x + roi.width/2, roi.y + roi.height/2),
                 cv::Scalar(0,255,255));
      }

      cv::rectangle(frame, roi, cv::Scalar(0,255,0), 1, 8, 0 );
      sprintf(message1,"dx=%04d, dy=%04d",dx, dy);
      putText(frame, message1, Point2f(20,30), FONT_HERSHEY_SIMPLEX, 0.75,  Scalar(0,255,0));
      putText(frame, message2, Point2f(20,60), FONT_HERSHEY_SIMPLEX, 0.75,  Scalar(0,255,0));
      cv::imshow(winName, frame);
    }
  }

  //vehicle->advancedSensing->stopMainCameraStream();

  if(tracker)
  {
    delete tracker;
  }

  return 0;
}
