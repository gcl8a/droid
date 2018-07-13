#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
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
  //ros::Subscriber sub;
  ros::Subscriber subEncoders; //subscriber for encoder data (of type droid::MotorData, gives speed in m/s?)
  ros::Publisher odom_pub;
    
  tf::TransformBroadcaster odom_broadcaster;

  ros::Duration t_delta;
  ros::Time t_next;
  ros::Time then;
    
  //wheel speeds (in m/s) reported from the encoder manager
  double vLeft;
  double vRight;
    
  //width of the robot (in m)
  double base_radius;
    
  double rate;
    
  double x_final, y_final, theta_final;
    
  ros::Time current_time, last_time;
    
  void encoderCallback(const droid::MotorData& wheelSpeeds);
    
  void init_variables();
  void get_node_params();
    
  void update();
};

Odometry_calc::Odometry_calc()
{
  init_variables();
    
  ROS_INFO("Started odometry computing node");
    
  subEncoders = nh.subscribe("encData", 10, &Odometry_calc::encoderCallback, this);
  odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50); //not sure I like this being asynchronous
    
  //Retrieving parameters of this node
  get_node_params();
}

void Odometry_calc::init_variables()
{
  rate = 10;

  base_radius = 0.227;
    
  t_delta = ros::Duration(1.0 / rate);
  t_next = ros::Time::now() + t_delta;
    
  then = ros::Time::now();
    
  vLeft = 0;
  vRight = 0;
    
  x_final = 0;
  y_final = 0;
  theta_final = 0;
    
  current_time = ros::Time::now();
  last_time = ros::Time::now();
}

void Odometry_calc::get_node_params()
{
  if(nh.getParam("rate", rate))
    {
      ROS_INFO_STREAM("Rate from param" << rate);
    }
    
  if(nh.getParam("base_radius", base_radius ))
    {
      ROS_INFO_STREAM("Base Radius" << base_radius );
    }
}

void Odometry_calc::spin()
{
  ros::Rate loop_rate(rate);
  while (ros::ok())
    {
      update();
      loop_rate.sleep();
    }
}

//Update function
void Odometry_calc::update()
{
  ros::Time now = ros::Time::now();
    
  if ( now > t_next)
    {
      double elapsed = now.toSec() - then.toSec();
        
      double u = (vLeft + vRight) / 2.0; //forward velocity in the robot frame
      double omega = (vRight - vLeft) / (2.0 * base_radius); //angular velocity

      // calculate the new position of the robot
      x_final = x_final + cos(theta_final) * u * elapsed;
      y_final = y_final + sin(theta_final) * u * elapsed;
        
      theta_final = theta_final + omega * elapsed;
        
      geometry_msgs::Quaternion odom_quat;
        
      odom_quat.x = 0.0;
      odom_quat.y = 0.0;
      odom_quat.z = 0.0;
        
      odom_quat.z = sin(theta_final / 2);
      odom_quat.w = cos(theta_final / 2);
        
      //first, we'll publish the transform over tf
      geometry_msgs::TransformStamped odom_trans;
      odom_trans.header.stamp = now;
      odom_trans.header.frame_id = "odom";
      odom_trans.child_frame_id = "base_link";
        
      odom_trans.transform.translation.x = x_final;
      odom_trans.transform.translation.y = y_final;
      odom_trans.transform.translation.z = 0.0;
      odom_trans.transform.rotation = odom_quat;
        
      //send the transform
      odom_broadcaster.sendTransform(odom_trans);
        
      //next, we'll publish the odometry message over ROS
      nav_msgs::Odometry odom;
      odom.header.stamp = now;
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
        
      then = now;

      ros::spinOnce();
    }

  else {}
}

void Odometry_calc::encoderCallback(const droid::MotorData& wheelSpeeds)
{
  vLeft  = wheelSpeeds.left;
  vRight = wheelSpeeds.right;
}

int main(int argc, char **argv)

{
  ros::init(argc, argv, "base_odom");
  Odometry_calc obj;
  obj.spin();
    
  return 0;
}
