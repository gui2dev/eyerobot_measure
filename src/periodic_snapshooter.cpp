#include <laser_assembler/AssembleScans2.h>
#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/Bool.h"

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Geometry>

using namespace laser_assembler;

ros::Subscriber b_sub;
ros::Publisher scan_pub;
ros::Time start_time, end_time;


      

bool bScan;
bool bNewScan;

void savePcl(pcl::PCLPointCloud2 &cloud)
{
	Eigen::Vector4f v = Eigen::Vector4f::Zero ();
Eigen::Quaternionf q = Eigen::Quaternionf::Identity ();

 std::stringstream ss;
      ss << "/home/gdev/Brosses/brosse_" << cloud.header.stamp << ".pcd";

      pcl::PCDWriter writer;

	      writer.writeBinary (ss.str (), cloud, v, q);
	      
      ROS_INFO ("Data saved to %s", ss.str ().c_str ());

}

void measureCallback(const std_msgs::Bool::ConstPtr &b)
{
  if (b->data)
  {
    start_time = ros::Time::now();
    bNewScan=true;
  }
  else
  {
    end_time = ros::Time::now();
  }

  bScan = b->data;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "client");
  ros::NodeHandle n;
  ros::Rate loop_rate(1);

  bScan = false;
  bNewScan=false;
  start_time = ros::Time::now();
  end_time = ros::Time::now();

  scan_pub = n.advertise<sensor_msgs::PointCloud2>("/my_cloud", 1);
  b_sub = n.subscribe<std_msgs::Bool>("/measure", 1, &measureCallback);
  ros::service::waitForService("assemble_scans2");

  ros::ServiceClient client = n.serviceClient<AssembleScans2>("assemble_scans2");

  AssembleScans2 srv;

  bool bFullPosted=true;
  while (ros::ok())
  {
    loop_rate.sleep();
    ros::spinOnce();
    if (bScan)
      end_time = ros::Time::now();

      srv.request.begin = start_time;
      srv.request.end = end_time;
      if (client.call(srv))
      {
        scan_pub.publish(srv.response.cloud);      
        if (!bScan && bNewScan)
        {
        pcl::PCLPointCloud2 cloud;
        pcl_conversions::toPCL(srv.response.cloud,cloud);
        savePcl(cloud);
        bNewScan=false;
	    }
	  }
        
        
  }
    

  return 0;
}
