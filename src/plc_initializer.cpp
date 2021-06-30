#include <string> // std::string, std::to_string()

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

#include <cognex_demo/plc_interface.h>

/**
 * @brief plc_initializer node
 * When the plc loses power, it wipes out the data storing the xyz locations of the keypoints
 * relative to the target.  These are time-consuming and error-prone to enter by hand.  As such,
 * this node will automatically populate them.
 */

int main(int argc, char **argv)
{
  // Start ROS
  ros::init(argc, argv, "plc_initializer");

  // Define geometric constants - these would be good to load from parameter server
  int rows = 5;                     // number of circles along 'x' direction
  int cols = 4;                     // number of circles along 'y' direction
  double circle_spacing = 0.03556;  // meters

  // Instantiate a plc interface
  demo::PLCInterface plci;

  // Declare variables that will be used over and over in the loops
  geometry_msgs::PoseStamped p;
  p.pose.orientation.w = 1.0;
  std::string node_id;
  int index;

  for (int y = 0; y < rows; ++y)
  {
    for (int x = 0; x < cols; ++x)
    {
      // Points on the target are traversed from -X to +X across each row
      // Rows are traversed in order from +Y to -Y
      index = cols * y + x;
      p.pose.position.x = circle_spacing * x;
      p.pose.position.y = circle_spacing * ((rows - 1) - y);

      // Write to PLC
      node_id = demo::node_ids::FEATURE_TF_PATH + std::to_string(index) + "]";
      plci.writeTag(node_id, p);
    }
  }

  ROS_INFO("Successfully initiated plc target pose transforms.");
  return 0;
}
