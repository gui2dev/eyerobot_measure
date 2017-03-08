#include <cmath>
#include "angles/angles.h"
#include "geometry_msgs/Quaternion.h"
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_srvs/Empty.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */

ros::Publisher q_pub;
ros::Publisher b_pub;
ros::Subscriber b_sub;
bool bScan, bZero;

double roll_scan_min, roll_scan_max, roll_scan_step, roll_offset;
double pitch_scan_min, pitch_scan_max, pitch_scan_step, pitch_offset;

void sendRPY(double roll, double pitch, double yaw)
{
	if (pitch + pitch_offset > 90 )
	 pitch = 90 - pitch_offset;
	 
	 	if (pitch + pitch_offset < -90 )
	 pitch = - 90 -pitch_offset  ;
	 
  geometry_msgs::Quaternion q;
  q = tf::createQuaternionMsgFromRollPitchYaw(angles::from_degrees(roll + roll_offset),
                                              angles::from_degrees(pitch + pitch_offset), angles::from_degrees(yaw));
  //  ROS_INFO("R:%.1f P:%.1f Y:%.1f",roll+ roll_offset,pitch+ pitch_offset,yaw);
  q_pub.publish(q);
  ros::spinOnce();
}

void scanRoll(double start, double end, double pitch, double yaw, double step, ros::Rate &loop_rate)
{
  ROS_INFO("From roll %.3f to roll %.3f at pitch %.3f by %.3f",start,end,pitch,step);
  double roll = start;
  if (start < end)
  {
    while (roll < end && ros::ok() && bScan)
    {
      sendRPY(roll, pitch, yaw);
      loop_rate.sleep();

      roll += step;
    }
  }
  else
  {
    while (roll > end && ros::ok() && bScan)
    {
      sendRPY(roll, pitch, yaw);
      loop_rate.sleep();


      roll -= step;
    }
  }
}

void scanPitch(double start, double end, double roll, double yaw, double step, ros::Rate &loop_rate)
{
	
  ROS_INFO("From pitch %.3f to pitch %.3f at roll %.3f by %.3f",start,end,roll,step);
  double pitch = start;
  if (start < end)
  {
    while (pitch < end && ros::ok() && bScan)
    {
      sendRPY(roll, pitch, yaw);
      loop_rate.sleep();

      pitch += step;
    }
  }
  else
  {
    while (pitch > end && ros::ok() && bScan)
    {
      sendRPY(roll, pitch, yaw);
      loop_rate.sleep();

      pitch -= step;
    }
  }
}

void getParam(std::string param_name_in, double &value, ros::NodeHandle &n)
{
  std::string param_name;
  if (n.searchParam(param_name_in, param_name))
  {
    // Found parameter, can now query it using param_name

    n.getParam(param_name, value);
  }
  else
  {
    ROS_INFO("No param %s found in an upward search", param_name_in);
  }
}

void measureCallback(const std_msgs::Bool::ConstPtr &b)
{
  bScan = b->data;
}

void calibrate(tf::Quaternion q)
{
  double roll, pitch, yaw;
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  bool bRollOk = false, bPitchOk = false;
  if (roll > angles::from_degrees(1))
    roll_offset -= 0.18;
  else if (roll < angles::from_degrees(-1))
    roll_offset += 0.18;
  else
    bPitchOk = true;

  if (pitch > angles::from_degrees(1))
    pitch_offset += 0.18;
  else if (pitch < angles::from_degrees(-1))
    pitch_offset -= 0.18;
  else

    bRollOk = true;

  bZero = (bPitchOk && bRollOk) ? true : false;

  ROS_INFO("Calibration %.3f, %.3f, %.3f --> %.3f , %.3f", angles::to_degrees(roll), angles::to_degrees(pitch),
           angles::to_degrees(yaw), roll_offset, pitch_offset);
}

void Proc1(ros::Rate &fast_rate, ros::Rate &slow_rate)
{
      scanRoll(0, roll_scan_min, 0, 0, roll_scan_step, fast_rate);

      scanRoll(roll_scan_min,roll_scan_max,  -5, 0, roll_scan_step, fast_rate);
      scanRoll(roll_scan_max,roll_scan_min,  -15, 0, roll_scan_step, fast_rate);
      scanRoll(roll_scan_min,roll_scan_max,  -20, 0, roll_scan_step, fast_rate);
      scanRoll(roll_scan_max,roll_scan_min,  -25, 0, roll_scan_step, fast_rate);
      scanRoll(roll_scan_min,roll_scan_max,  -30, 0, roll_scan_step, fast_rate);
      scanRoll(roll_scan_max,roll_scan_min,  -35, 0, roll_scan_step, fast_rate);
      scanRoll(roll_scan_min,roll_scan_max,  -40, 0, roll_scan_step, fast_rate);
      scanRoll(roll_scan_max,roll_scan_min,  -45, 0, roll_scan_step, fast_rate);


      scanPitch(-45, 5, roll_scan_min, 0, pitch_scan_step, fast_rate);

      scanRoll(roll_scan_min,roll_scan_max,  5, 0, roll_scan_step, fast_rate);
      scanRoll(roll_scan_max,roll_scan_min,  15, 0, roll_scan_step, fast_rate);
      scanRoll(roll_scan_min,roll_scan_max,  20, 0, roll_scan_step, fast_rate);
      scanRoll(roll_scan_max,roll_scan_min,  25, 0, roll_scan_step, fast_rate);
      scanRoll(roll_scan_min,roll_scan_max,  30, 0, roll_scan_step, fast_rate);
      scanRoll(roll_scan_max,roll_scan_min,  35, 0, roll_scan_step, fast_rate);
      scanRoll(roll_scan_min,roll_scan_max,  40, 0, roll_scan_step, fast_rate);
      scanRoll(roll_scan_max,roll_scan_min,  45, 0, roll_scan_step, fast_rate);


      scanPitch(45, 0, roll_scan_min, 0, pitch_scan_step, fast_rate);

      scanRoll(roll_scan_min, 0, 0, 0, roll_scan_step, slow_rate);

      scanPitch(0, pitch_scan_min, 0, 0, pitch_scan_step, fast_rate);

      scanPitch(pitch_scan_min,pitch_scan_max, -5, 0, pitch_scan_step, fast_rate);
      scanPitch(pitch_scan_max,pitch_scan_min, -15, 0, pitch_scan_step, fast_rate);
      scanPitch(pitch_scan_min,pitch_scan_max, -25, 0, pitch_scan_step, fast_rate);


      scanRoll(-25,5,  pitch_scan_min, 0, roll_scan_step, fast_rate);

      scanPitch(pitch_scan_min,pitch_scan_max, 5, 0, pitch_scan_step, fast_rate);
      scanPitch(pitch_scan_max,pitch_scan_min, 15, 0, pitch_scan_step, fast_rate);
      scanPitch(pitch_scan_min,pitch_scan_max, 25, 0, pitch_scan_step, fast_rate);

      scanRoll(25,0,  pitch_scan_min, 0, roll_scan_step, fast_rate);

      scanPitch(pitch_scan_min, pitch_scan_max, 0, 0, pitch_scan_step, slow_rate);

      scanPitch(pitch_scan_max, 0, 0, 0, pitch_scan_step, fast_rate);

      scanRoll(0, roll_scan_max, 0, 0, roll_scan_step, slow_rate);

      scanRoll(roll_scan_max, 0, 0, 0, roll_scan_step, fast_rate);
}

void Proc2(ros::Rate &fast_rate, ros::Rate &slow_rate)
{
//scanRoll(0,roll_scan_min,  0, 0, roll_scan_step, fast_rate);
//scanPitch(0, pitch_scan_max, 0, 0, roll_scan_step, fast_rate);
//scanPitch(pitch_scan_max, pitch_scan_min, 0, 0, roll_scan_step, fast_rate);
//scanRoll(0,roll_scan_min,  pitch_scan_min, 0, roll_scan_step, fast_rate);
double pitch=pitch_scan_min;
while (pitch<pitch_scan_max && ros::ok() && bScan)
{
	  scanRoll(roll_scan_max,15,pitch,0,roll_scan_step,fast_rate);
      scanRoll(15,5,  pitch, 0, roll_scan_step, slow_rate);
      scanRoll(5,0,  pitch, 0, roll_scan_step, slow_rate);
      pitch+=pitch_scan_step;
      scanRoll(0,5,  pitch, 0, roll_scan_step, slow_rate);
      scanRoll(5,15,  pitch, 0, roll_scan_step, slow_rate);
	  scanRoll(15,roll_scan_max,pitch,0,roll_scan_step,fast_rate);
      pitch+=pitch_scan_step;
}
}

void Proc3(ros::Rate &fast_rate, ros::Rate &slow_rate)
{
	scanRoll(0,roll_scan_max,  0, 0, roll_scan_step, fast_rate);
	scanRoll(roll_scan_max,25,0,0,roll_scan_step,fast_rate);
	scanPitch(0,pitch_scan_min,  25, 0, roll_scan_step, fast_rate);
	double pitch=pitch_scan_min;
while (pitch<pitch_scan_max && ros::ok() && bScan)
{
	scanRoll(25,-25,pitch,0,roll_scan_step,fast_rate);
	pitch+=pitch_scan_step;
	scanRoll(-25,25,pitch,0,roll_scan_step,fast_rate);
		pitch+=pitch_scan_step;

}
 scanPitch(pitch_scan_max,0,25,0,roll_scan_step,fast_rate);
 scanRoll(25,0,0,0,roll_scan_step,fast_rate);
}
	
int main(int argc, char **argv)
{
  bScan = false;
  bZero = false;
  std_msgs::BoolPtr msg(new std_msgs::Bool);
  msg->data = false;
  roll_scan_min = -25;
  roll_scan_max = 25;
  roll_scan_step = 1;
  roll_offset = 0;

  pitch_scan_min = -45;
  pitch_scan_max = 45;
  pitch_scan_step = 1;
  pitch_offset = 0;

  ros::init(argc, argv, "scan_proc");

  ros::NodeHandle n;

  q_pub = n.advertise<geometry_msgs::Quaternion>("/orientation", 10);

  b_sub = n.subscribe<std_msgs::Bool>("/measure", 10, &measureCallback);

  b_pub = n.advertise<std_msgs::Bool>("/measure", 10);

  getParam("/scan_proc/roll_scan_min", roll_scan_min, n);
  getParam("/scan_proc/roll_scan_max", roll_scan_max, n);
  getParam("/scan_proc/roll_scan_step", roll_scan_step, n);

  getParam("/scan_proc/pitch_scan_min", pitch_scan_min, n);
  getParam("/scan_proc/pitch_scan_max", pitch_scan_max, n);
  getParam("/scan_proc/pitch_scan_step", pitch_scan_step, n);

  ros::service::waitForService("stop_motor");
  ros::ServiceClient clientStop = n.serviceClient<std_srvs::Empty>("stop_motor");

  ros::service::waitForService("start_motor");
  ros::ServiceClient clientStart = n.serviceClient<std_srvs::Empty>("start_motor");

  std_srvs::Empty srvEmpty;
  clientStop.call(srvEmpty);

  ros::Rate slow_rate(5);
  ros::Rate fast_rate(50);
  
  
  ros::Duration(5).sleep();

  ROS_INFO("Starting calibration");

  tf::TransformListener listener;
  tf::StampedTransform transform;
  ros::Duration(5).sleep();
  while (ros::ok() && !bZero)
  {
    try
    {
      listener.lookupTransform("/base_link", "/laser", ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }
    calibrate(transform.getRotation());

    sendRPY(0, 0, 0);
    ros::spinOnce();
    fast_rate.sleep();
  }
  ROS_INFO("Calibration ok");

  while (ros::ok())
  {
    ros::spinOnce();
    fast_rate.sleep();
    if (bScan)
    {
      clientStart.call(srvEmpty);
      ros::Duration(5).sleep();
    }

    while (ros::ok() && bScan)
    {
      ros::spinOnce();
//      Proc1(fast_rate,slow_rate);
      Proc3(fast_rate,slow_rate);
      b_pub.publish(msg);
      ros::spinOnce();
    }
    sendRPY(0, 0, 0);

    clientStop.call(srvEmpty);
  }

  return 0;
}


