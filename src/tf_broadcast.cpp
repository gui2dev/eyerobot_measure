#include <angles/angles.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#define r4 0.0255
#define r3 0.031
#define r2 0.0127
#define gx 0.0115
#define gy 0.03139

#define r1 0.033

#define theta1 1.937

double calcBeta(double theta2)
{
  double x3 = r1 * cos(theta1) - r2 * cos(theta2);
  double y3 = r1 * sin(theta1) - r2 * sin(theta2);

  double theta41 = atan2(y3, x3);
  double theta42 = acos((x3 * x3 + y3 * y3 + r2 * r2 - r1 * r1) / (-2 * r2 * sqrt(x3 * x3 + y3 * y3)));

  ROS_INFO("%.3f --> %.3f or %.3f", angles::to_degrees(theta2), angles::to_degrees(theta41 + theta42),
           angles::to_degrees(theta41 - theta42));

  return (theta41 - theta42);
}

void poseCallback(const sensor_msgs::Imu::ConstPtr& imu_data)
{
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q(imu_data->orientation.x,imu_data->orientation.y,imu_data->orientation.z,imu_data->orientation.w);
  double roll, pitch, yaw;
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  //ROS_INFO("R: %.3f P: %.3f Y: %.3f", angles::to_degrees(roll), angles::to_degrees(pitch), angles::to_degrees(yaw));
  //pitch = calcBeta(-pitch);
  transform.setOrigin(tf::Vector3(0, 0, 0.16));
  // transform.setOrigin( tf::Vector3(0,0,0));
  q.setRPY(pitch, 0, 0);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "gimbal_r"));
  transform.setOrigin(tf::Vector3(0, 0, 0.0342));
  q.setRPY(0, -roll, 0);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "gimbal_r", "gimbal_p"));
  transform.setOrigin(tf::Vector3(-0.01121, 0, 0.00229));
  q.setRPY(0, 0, angles::from_degrees(180));
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "gimbal_p", "laser"));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_broadcast");

  ros::NodeHandle node;

  ros::Subscriber sub = node.subscribe("/imu/data", 100, &poseCallback);

  ros::spin();
  return 0;
};
