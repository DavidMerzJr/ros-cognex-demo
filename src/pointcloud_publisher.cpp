#include <string>

#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl_ros/point_cloud.h>
#include <ros/package.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pointcloud_publisher");
  ros::NodeHandle nh;

  // Get parameters
  std::string filename, frame;
  nh.getParam("pointcloud_file", filename);
  nh.getParam("pointcloud_frame", frame);

  // Load the pointcloud
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  pcl::PLYReader file_loader;
  file_loader.read(filename, cloud);
  cloud.header.frame_id = frame;
  for (pcl::PointXYZRGB& p : cloud)
  {
    p.x /= 1000;
    p.y /= 1000;
    p.z /= 1000;
  }

  // Make a publisher
  ros::Publisher pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("point_cloud", 1, true);

  //  // Re-publish occasionally in case the frame moves
  //  ros::Rate loop_rate(0.01);
  //  while (ros::ok())
  //  {
  pub.publish(cloud);
  //    ros::spinOnce();
  //    loop_rate.sleep();
  //  }

  ros::spin();
  return 0;
}
