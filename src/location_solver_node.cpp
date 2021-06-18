#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "location_solver_node");
  ros::NodeHandle nh;

  ros::Publisher location_pub = nh.advertise<geometry_msgs::PoseStamped>("/pose_relative_to_camera", 3);

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    geometry_msgs::PoseStamped msg;
    msg.header.frame_id = "map";
    msg.pose.position.x = 1.0;
    msg.pose.position.y = 2.0;
    msg.pose.position.z = 3.0;
    msg.pose.orientation.w = 1.0;

    location_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}