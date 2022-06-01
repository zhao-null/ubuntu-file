#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Path.h> 
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h> 
#include <tf/transform_broadcaster.h>
#include <sstream>
#include <string>
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <math.h>
#include "serial/serial.h"

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;
unsigned char speed_data[9] = {0};//发送给串口的数据
float ratio = 1000.0f;//转速转换比例，执行速度调整比例
float WheelTread = 0.24f;//两轮间距，单位是m;
float linear_temp = 0, angular_temp = 0;//暂存的线速度和角速度
int left_speed_data_d = 0;
int right_speed_data_d = 0;
serial::Serial my_serial;

void OdomParse(std::string s ,double& vx,double& vth)
{
  std::string x;
  std::string th;
  int num_s = 0;
  int num_m = 0;
  int num_e = 0;
  num_s = s.find("s");
  num_m = s.find("m");
  num_e = s.find("e"); 
  x = s.substr(num_s+1,num_m-num_s-1);
  th = s.substr(num_m+1,num_e-num_m-1);
  vx = atof(x.c_str());
  vth = atof(th.c_str());
}

void my_sleep(unsigned long milliseconds)
{
      usleep(milliseconds*1000); // 100 ms
}

void enumerate_ports()
{
    vector<serial::PortInfo> devices_found = serial::list_ports();

    vector<serial::PortInfo>::iterator iter = devices_found.begin();

    while( iter != devices_found.end() )
    {
        serial::PortInfo device = *iter++;
        printf( "(%s, %s, %s)\n", device.port.c_str(), device.description.c_str(),
        device.hardware_id.c_str() );
    }
}

void callback(const geometry_msgs::Twist& cmd_input)//订阅 /cmd_vel话题回调函数
{
  ROS_INFO_STREAM("subscribe the cmd_vel node\n"); 
  angular_temp = cmd_input.angular.z;//获取/cmd_vel角速度,rad/s
  linear_temp = cmd_input.linear.x;//获取/cmd_vel线速度，m/s
  //将角速度和线速度转换成两个车轮的速度
  left_speed_data_d = (linear_temp - 0.5f*angular_temp*WheelTread)*ratio;
  right_speed_data_d = (linear_temp + 0.5f*angular_temp*WheelTread)*ratio;

  cout<<"left  wheel speed is :"<<left_speed_data_d<<"mm/s"<<endl;
  cout<<"right wheel speed is :"<<right_speed_data_d<<"mm/s"<<endl;

  speed_data[1] = abs(left_speed_data_d) / 100 + '0';
  speed_data[2] = abs(left_speed_data_d) % 100 /10 + '0';
  speed_data[3] = abs(left_speed_data_d) % 10 + '0' ;

  speed_data[5] = abs(right_speed_data_d) / 100 + '0';
  speed_data[6] = abs(right_speed_data_d) % 100 /10 + '0';
  speed_data[7] = abs(right_speed_data_d) % 10 + '0';
  if(left_speed_data_d<0){
    speed_data[0] = '-';
  }else{
    speed_data[0] = '+';
  }
  if(right_speed_data_d<0){
    speed_data[4] = '-';
  }else{
    speed_data[4] = '+';
  }
  speed_data[8] = '?';
  if(my_serial.isOpen())
    {
      my_serial.write(speed_data, 9);
      cout<<speed_data[0]<<speed_data[1]<<speed_data[2]<<speed_data[3]<<speed_data[4]<<speed_data[5]<<speed_data[6]<<speed_data[7]<<speed_data[8]<<endl;
      ROS_INFO_STREAM("Serial Port is OK, data is sending......");
    }
}


void print_usage()
{
    cerr << "Usage: test_serial {-e|<serial port address>} ";
    cerr << "<baudrate> [test string]" << endl;
}

int run(int argc, char **argv)
{
  if(argc < 2) {
      print_usage();
    return 0;
  }
  string port(argv[1]);
  unsigned long baud = 0;
  sscanf(argv[2], "%lu", &baud);
  my_serial.setPort(port);
  my_serial.setBaudrate(baud);
  serial::Timeout to = serial::Timeout::simpleTimeout(1000);
  my_serial.setTimeout(to);
  try
  {
      my_serial.open();
  }
  catch(serial::IOException &e)
  {
      ROS_ERROR_STREAM("Unable to open port");
      return -1;
  }
  if(my_serial.isOpen())
  {
      ROS_INFO_STREAM("Serial Port Initialized");
  }
  else
      return -1;
}

int main(int argc, char **argv) {
  my_sleep(5);
  run(argc, argv);
  ros::init(argc,argv,"serial_example");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("cmd_vel", 1, callback);//订阅 /cmd_vel话题
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 20);
  ros::Publisher path_pub = n.advertise<nav_msgs::Path>("trajectory",1,true);
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  tf::TransformBroadcaster odom_broadcaster;
  std::string result; 
  nav_msgs::Path path;
  path.header.stamp=current_time;
  path.header.frame_id="odom";
  double x = 0.0;
  double y = 0.0;
  double w = 0.0;
  double th = 0.0;
  ros::Rate loop_rate(50);
  while (ros::ok())
  {
    if(my_serial.available())
    {
      ROS_INFO_STREAM("Reading from serial port\n"); 
      result += my_serial.read(my_serial.available());
      std::string ostart = "s";
      std::string oend = "e";
      int i = 0, start = -1, end = -1;
      while(i < result.length())
      {
        start = result.find(ostart);
        if(start == -1)
        {
          result = result.substr(result.length());
          break;
        }else{
          end = result.find(oend);
          if(end == -1){
              result = result.substr(start);
              break;
            }
            else{
              i = end;
              double vx,vth;
              double vy =0;
              OdomParse(result.substr(start,end-start+1),vx,vth);
              current_time = ros::Time::now();
              double dt = (current_time - last_time).toSec();
              x += vx*cos(th+vth/2)*dt;// x方向前进的距离，单位m
              y += vx*sin(th+vth/2)*dt;//y方向前进的距离，单位m
              th +=vth*dt;//角度w，单位rad
              geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
              //first, we'll publish the transform over tf
              geometry_msgs::TransformStamped odom_trans;
              odom_trans.header.stamp = current_time;
              odom_trans.header.frame_id = "odom";
              odom_trans.child_frame_id = "base_link";

              odom_trans.transform.translation.x = x/1000;
              odom_trans.transform.translation.y = y/1000;
              odom_trans.transform.translation.z = 0.0;
              odom_trans.transform.rotation = odom_quat;

              //send the transform
              odom_broadcaster.sendTransform(odom_trans);

              //next, we'll publish the odometry message over ROS
              nav_msgs::Odometry odom;
              odom.header.stamp = current_time;
              odom.header.frame_id = "odom";

              //set the position
              odom.pose.pose.position.x = x/1000;
              odom.pose.pose.position.y = y/1000;
              odom.pose.pose.position.z = 0.0;
              odom.pose.pose.orientation = odom_quat;

              //set the velocity
              odom.child_frame_id = "base_link";
              odom_pub.publish(odom);

              geometry_msgs::PoseStamped this_pose_stamped;
              this_pose_stamped.pose.position.x = x/1000; 
		          this_pose_stamped.pose.position.y = y/1000; 
              this_pose_stamped.pose.orientation.x = odom_quat.x; 
              this_pose_stamped.pose.orientation.y = odom_quat.y; 
              this_pose_stamped.pose.orientation.z = odom_quat.z; 
              this_pose_stamped.pose.orientation.w = odom_quat.w; 
              this_pose_stamped.header.stamp=current_time;
              this_pose_stamped.header.frame_id="odom";
              path.poses.push_back(this_pose_stamped);

              path_pub.publish(path);

              //publish the message
              last_time = current_time;
              result = result.substr(end+1);
            }
        }
      }
    } 
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0 ;
}
