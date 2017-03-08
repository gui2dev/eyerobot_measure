#include "laser_geometry/laser_geometry.h"
#include "message_filters/subscriber.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/Bool.h"
#include "tf/message_filter.h"
#include "tf/transform_listener.h"

class LaserScanToPointCloud
{
public:
  ros::NodeHandle n_;
  laser_geometry::LaserProjection projector_;
  tf::TransformListener listener_;
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
  tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
  ros::Publisher scan_pub_;
  ros::Subscriber b_sub_;
  std::string frame_id;
  bool bScan;

  LaserScanToPointCloud(ros::NodeHandle n, std::string value)
    : n_(n), laser_sub_(n_, "scan", 100), laser_notifier_(laser_sub_, listener_, "laser", 100), bScan(false),frame_id(value)
  {
    laser_notifier_.registerCallback(boost::bind(&LaserScanToPointCloud::scanCallback, this, _1));
    laser_notifier_.setTolerance(ros::Duration(0.1));
    scan_pub_ = n_.advertise<sensor_msgs::PointCloud2>("/my_cloud", 1);
    b_sub_ =
        n_.subscribe<std_msgs::Bool>("/measure", 1, boost::bind(&LaserScanToPointCloud::measureCallback, this, _1));
  }

  void measureCallback(const std_msgs::Bool::ConstPtr& b_in)
  {
    bScan = b_in->data;
  }

  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
  {
    if (bScan)
    {
      sensor_msgs::PointCloud2 cloud;
      try
      {
        projector_.transformLaserScanToPointCloud(frame_id, *scan_in, cloud, listener_);
      }
      catch (tf::TransformException& e)
      {
        std::cout << e.what();
        return;
      }

      scan_pub_.publish(cloud);
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "scan_to_cloud");
  ros::NodeHandle n;
  std::string value,param_name;
  if (n.searchParam("static_frame", param_name))
  {
    // Found parameter, can now query it using param_name

    n.getParam(param_name, value);
  }
  else
  {
    ROS_INFO("No param static_frame found in an upward search");
  }

  LaserScanToPointCloud lstopc(n,value);

  ros::spin();

  return 0;
}
