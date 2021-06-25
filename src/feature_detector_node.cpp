#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cognex_demo/plc_interface.h>

void uploadToPLC(const std::vector<std::pair<int, int>>& uv_coords)
{
  // upload to PLC
  return;
}

void imageCallback(const sensor_msgs::Image::ConstPtr& image)
{
  ROS_INFO_STREAM("Received Image");
  // std::vector<std::pair<int, int>> uv_coords = findCircles(image);
  // uploadToPLC(uv_coords);
  return;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "feature_detector_node");
  ros::NodeHandle nh;

  //plc_interface_.init();

  ros::Subscriber image_sub = nh.subscribe("pylon_camera_node/image_raw", 5, imageCallback);

  ros::spin();
  return 0;
}
