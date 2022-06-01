#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
 
#include <nav_msgs/Odometry.h>
 
 
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
 
 
nav_msgs::Path  slave_path;
nav_msgs::Path  master_path;
 
ros::Publisher  slave_path_pub;
ros::Publisher  master_path_pub;
ros::Subscriber slaveOdomSub;
ros::Subscriber masterOdomSub;
 
 void odomCallback(const nav_msgs::Odometry::ConstPtr& odom)
 {
    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.pose.position.x = odom->pose.pose.position.x;
    this_pose_stamped.pose.position.y = odom->pose.pose.position.y;
 
    this_pose_stamped.pose.orientation = odom->pose.pose.orientation;
 
    this_pose_stamped.header.stamp = ros::Time::now();
    this_pose_stamped.header.frame_id = "slave/odom";
 
    slave_path.poses.push_back(this_pose_stamped);
 
    slave_path.header.stamp = ros::Time::now();
    slave_path.header.frame_id="slave/odom";
    slave_path_pub.publish(slave_path);
    // printf("path_pub ");
 
    // printf("odom %.3lf %.3lf\n",odom->pose.pose.position.x,odom->pose.pose.position.y);
 }
 void masterOdomCallback(const nav_msgs::Odometry::ConstPtr& odom)
 {
    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.pose.position.x = odom->pose.pose.position.x;
    this_pose_stamped.pose.position.y = odom->pose.pose.position.y;
 
    this_pose_stamped.pose.orientation = odom->pose.pose.orientation;
 
    this_pose_stamped.header.stamp = ros::Time::now();
    this_pose_stamped.header.frame_id = "slave/odom";
 
    master_path.poses.push_back(this_pose_stamped);
 
    master_path.header.stamp = ros::Time::now();
    master_path.header.frame_id="slave/odom";
    master_path_pub.publish(master_path);
    // printf("path_pub ");
 
    // printf("odom %.3lf %.3lf\n",odom->pose.pose.position.x,odom->pose.pose.position.y);
 }
 
int main (int argc, char **argv)
{
    ros::init (argc, argv, "showpath");
 
    ros::NodeHandle ph;
 
    slave_path_pub = ph.advertise<nav_msgs::Path>("slave_trajectory",10, true);
    master_path_pub = ph.advertise<nav_msgs::Path>("master_trajectory",10, true);

    slaveOdomSub  = ph.subscribe<nav_msgs::Odometry>("slave/odom", 10, odomCallback);
    masterOdomSub  = ph.subscribe<nav_msgs::Odometry>("master/odom", 10, masterOdomCallback);
 
 
    ros::Rate loop_rate(50);
 
    while (ros::ok())
    {
        ros::spinOnce();               // check for incoming messages
        loop_rate.sleep();
    }
 
    return 0;
}
