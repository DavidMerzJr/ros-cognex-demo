#include <string>

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>

namespace demo
{

const static std::string cam_info_topic = "/pylon_camera_node/camera_info";
const static std::string param_ns = "/camera_info/";

} // namespace demo

int main(int argc, char **argv)
{
  ros::init(argc, argv, "camera_info_publisher_node");
  ros::NodeHandle nh;

  sensor_msgs::CameraInfo cam_info;
  bool success = true;

  {
    int height, width;
    success = nh.getParam(demo::param_ns + "image_height", height);
    cam_info.height = height;
    success &= nh.getParam(demo::param_ns + "image_width", width);
    cam_info.width = width;

    std::string model;
    success &= nh.getParam(demo::param_ns + "distortion_model", model);
    cam_info.distortion_model = model;
    success &= nh.getParam(demo::param_ns + "distortion_coefficients/data", cam_info.D);
    std::vector<double> data;
    if (nh.getParam(demo::param_ns + "camera_matrix/data", data) && data.size() == 9)
      std::copy(data.begin(), data.end(), cam_info.K.begin());
    else
      success = false;
    if (nh.getParam(demo::param_ns + "rectification_matrix/data", data) && data.size() == 9)
      std::copy(data.begin(), data.end(), cam_info.R.begin());
    else
      success = false;
    if (nh.getParam(demo::param_ns + "projection_matrix/data", data) && data.size() == 12)
      std::copy(data.begin(), data.end(), cam_info.P.begin());
    else
      success = false;
  }

  if (success)
  {
    ros::Duration sleep(30.0);
    ros::Publisher cam_info_pub = nh.advertise<sensor_msgs::CameraInfo>(demo::cam_info_topic, 1000, true);

    while (ros::ok())
    {
      cam_info_pub.publish(cam_info);
      ros::spinOnce();
      sleep.sleep();
    }
  }

  return 0;
}
