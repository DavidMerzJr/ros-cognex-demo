#include <ros/ros.h>
#include <sensor_msgs/Image.h>

void uploadToPLC(const std::vector<std::pair<int, int>>& uv_coords)
{
  // upload to PLC
  return;
}

void imageCallback(const sensor_msgs::Image::ConstPtr& image)
{
  ROS_INFO("Received Image");
  // std::vector<std::pair<int, int>> uv_coords = findCircles(image);
  // uploadToPLC(uv_coords);
  return;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "feature_detector_node");
  ros::NodeHandle nh;

  ros::Subscriber image_sub = nh.subscribe("/camera/rgb", 5, imageCallback);

  ros::spin();
  return 0;
}
