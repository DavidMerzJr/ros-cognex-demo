#include <ros/ros.h>
#include <sensor_msgs/Image.h>

void imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  ROS_INFO("Received Image");
  return;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "feature_detector_node");
  ros::NodeHandle nh;

  ros::Subscriber image_sub = nh.subscribe("chatter", 5, imageCallback);

  ros::spin();
  return 0;
}
