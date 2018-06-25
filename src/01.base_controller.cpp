#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <droid/MotorData.h>

class MotorManager
{
private:
  ros::NodeHandle nh;
  ros::Subscriber subCmdVel;
  ros::Publisher pubMotorTargets;

  //not really sure what these are all here for
  ros::Duration t_delta;
  ros::Time t_next;
  ros::Time then;
  ros::Time current_time, last_time;
    
  double base_radius;
  double rate;

  double vLeft;
  double vRight;

private:
  void CmdVelCallback(const geometry_msgs::Twist& cmd_vel);

  void Init(void);
  void GetParams(void);
  void Update(void);

public:
  MotorManager(void);
  void Spin(void);
};

MotorManager::MotorManager(void)
{
  Init();
  GetParams();

  ROS_INFO("Started CmdVel node.");

  //subscribe to Twist messages on cmd_vel
  subCmdVel = nh.subscribe("cmd_vel", 10, &MotorManager::CmdVelCallback, this);

  //publish the motor speeds
  pubMotorTargets = nh.advertise<droid::MotorData>("motor_targets", 10);
}

void MotorManager::Init(void)
{
  rate = 10;

  base_radius = 0.18;

  vLeft = 0;
  vRight = 0;
    
  t_delta = ros::Duration(1.0 / rate);
  t_next = ros::Time::now() + t_delta;
    
  then = ros::Time::now();
    
  current_time = ros::Time::now();
  last_time = ros::Time::now();
}

void MotorManager::GetParams(void)
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

void MotorManager::Spin(void)
{
  ros::Rate loop_rate(rate);
  while (ros::ok())
    {
      Update();
      loop_rate.sleep();
    }
}

void MotorManager::Update(void)
{
  ros::Time now = ros::Time::now();
    
  if(now > t_next)
    {
      double elapsed = now.toSec() - then.toSec();
        
      droid::MotorData motorSpeeds;
      motorSpeeds.mode  = 1;
      motorSpeeds.left  = vLeft;
      motorSpeeds.right = vRight;

      pubMotorTargets.publish(motorSpeeds);

      then = now;

      ros::spinOnce();
    }

  else {}
}

void MotorManager::CmdVelCallback(const geometry_msgs::Twist& cmd_vel)
{
  double u = cmd_vel.linear.x;
  double omega = cmd_vel.angular.z;

  vLeft  = u - omega * base_radius;
  vRight = u + omega * base_radius;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "base_controller");

  MotorManager manager;
  manager.Spin();

  return 0;
}
