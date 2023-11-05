#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt32.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <droid/MotorData.h>


class Odometry_calc
{
public:
  Odometry_calc();
  void spin();

private:
  ros::NodeHandle nh;
  ros::Subscriber subEncoders; //subscriber for encoder data (of type droid::MotorData, gives speed in mrad/s)
  ros::Publisher odom_pub;
    
  tf::TransformBroadcaster odom_broadcaster;

  //wheel speeds (in mrad/s) reported from uC 
  double vLeft;
  double vRight;
    
  //width of the robot (in m)
  double base_radius;
  double wheel_radius;
    
  double x_final, y_final, theta_final;

  ros::Time curr_time, last_time;
    
  void encoderCallback(const std_msgs::UInt32& wheelSpeeds);
    
  void init_variables();
  void get_node_params();
};

Odometry_calc::Odometry_calc()
{
  init_variables();
    
  ROS_INFO("Started odometry computing node");
    
  subEncoders = nh.subscribe("motor_speeds", 100, &Odometry_calc::encoderCallback, this);
  odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50); 
    
  //Retrieving parameters of this node
  get_node_params();
}

void Odometry_calc::init_variables()
{
  base_radius = 0.227; // needs to agree with base_controller
  wheel_radius = 0.08;

  vLeft = 0;
  vRight = 0;
    
  x_final = 0;
  y_final = 0;
  theta_final = 0;
    
  curr_time = ros::Time::now();
  last_time = ros::Time::now();
}

void Odometry_calc::get_node_params()
{
  // if(nh.getParam("rate", rate))
  //   {
  //     ROS_INFO_STREAM("Rate from param" << rate);
  //   }
    
  if(nh.getParam("base_radius", base_radius ))
    {
      ROS_INFO_STREAM("Base Radius" << base_radius );
    }
}

void Odometry_calc::spin()
{
  while (ros::ok())
  {
    ros::spinOnce();
  }
}

// wheelSpeeds is in mrad/sec
void Odometry_calc::encoderCallback(const std_msgs::UInt32& wheelSpeeds)
{
  curr_time = ros::Time::now();

  int16_t speeds[2];
  memcpy(speeds, &wheelSpeeds, 4);

  double elapsed = (curr_time - last_time).toSec();

  // vLeft, vRight in m/s    
  float vLeft = speeds[0] * wheel_radius / 1000.;
  float vRight = speeds[1] * wheel_radius / 1000.;
        
      double u = (vLeft + vRight) / 2.0; // forward velocity in the robot frame
      double omega = (vRight - vLeft) / (2.0 * base_radius); // angular velocity

      // calculate the new pose of the robot
      double theta = theta_final;  //theta from prev iteration
      theta_final = theta_final + omega * elapsed;

      x_final = x_final + cos((theta + theta_final) / 2.0) * u * elapsed;
      y_final = y_final + sin((theta + theta_final) / 2.0) * u * elapsed;

      geometry_msgs::Quaternion odom_quat;
        
      odom_quat.x = 0.0;
      odom_quat.y = 0.0;
        
      odom_quat.z = sin(theta_final / 2);
      odom_quat.w = cos(theta_final / 2);
        
      // publish the transform over tf
      geometry_msgs::TransformStamped odom_trans;
      odom_trans.header.stamp = curr_time;
      odom_trans.header.frame_id = "odom";
      odom_trans.child_frame_id = "base_link";
        
      odom_trans.transform.translation.x = x_final;
      odom_trans.transform.translation.y = y_final;
      odom_trans.transform.translation.z = 0.0;
      odom_trans.transform.rotation = odom_quat;
        
      //send the transform
      odom_broadcaster.sendTransform(odom_trans);
        
      // publish the odometry message over ROS
      nav_msgs::Odometry odom;
      odom.header.stamp = curr_time;
      odom.header.frame_id = "odom";

      //set the position
      odom.pose.pose.position.x = x_final;
      odom.pose.pose.position.y = y_final;
      odom.pose.pose.position.z = 0.0;
      odom.pose.pose.orientation = odom_quat;
        
      //set the velocity
      odom.child_frame_id = "base_link";
      odom.twist.twist.linear.x = u;
      odom.twist.twist.linear.y = 0;
      odom.twist.twist.angular.z = omega;
        
      //publish the message
      odom_pub.publish(odom);

      last_time = curr_time;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "odometry_manager");
  Odometry_calc odom;
  odom.spin();
    
  return 0;
}
