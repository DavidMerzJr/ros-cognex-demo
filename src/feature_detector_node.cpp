#include <string>   // std::string
#include <utility>  // std::pair
#include <vector>   // std::vector

#include <opc/ua/client/client.h>
#include <opc/ua/subscription.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <cognex_demo/plc_interface.h>

namespace demo
{

class FeatureDetector
{
public:
  FeatureDetector(ros::NodeHandle& nh)
    : nh_ (nh)
  {
    image_sub_ = nh.subscribe("pylon_camera_node/image_raw", 5, &FeatureDetector::imageCallback, this);
    return;
  }

  void imageCallback(const sensor_msgs::Image::ConstPtr& image)
  {
    std::vector<std::pair<float, float>> uv_coords; // = findCircles(image);
    plci_.writeTag(node_ids::FEATURE_UV_PATH, uv_coords);
    return;
  }

private:
  ros::NodeHandle nh_;
  PLCInterface plci_;
  ros::Subscriber image_sub_;
};

} // namespace demo

int main(int argc, char **argv)
{
  ros::init(argc, argv, "feature_detector_node");
  ros::NodeHandle nh;

  demo::FeatureDetector fd(nh);

  ros::spin();
  return 0;
}
