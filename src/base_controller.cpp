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
  ros::Time lastCmdVel;
  ros::Duration CMD_VEL_TIMEOUT;

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
  //these can be changed by params?
  rate = 10;
  base_radius = 0.18;

  vLeft = 0;
  vRight = 0;
    
  CMD_VEL_TIMEOUT = ros::Duration(1); //1 sec timeout
  lastCmdVel = ros::Time::now();
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

      //if we haven't heard from the CmdVel topic in a while, cut the motors
      if(ros::Time::now() > lastCmdVel + CMD_VEL_TIMEOUT)
	{
	  vLeft = vRight = 0;
	}
    }
}

void MotorManager::Update(void)
{
  droid::MotorData motorSpeeds;

  motorSpeeds.mode  = 1;
  motorSpeeds.left  = vLeft;
  motorSpeeds.right = vRight;
  
  pubMotorTargets.publish(motorSpeeds);

  ros::spinOnce();
}

void MotorManager::CmdVelCallback(const geometry_msgs::Twist& cmd_vel)
{
  double u = cmd_vel.linear.x;
  double omega = cmd_vel.angular.z;

  vLeft  = u - omega * base_radius;
  vRight = u + omega * base_radius;

  lastCmdVel = ros::Time::now();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "base_controller");

  MotorManager manager;
  manager.Spin();

  return 0;
}
