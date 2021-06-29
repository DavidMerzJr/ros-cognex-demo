#include <memory>   // std::shared_ptr
#include <string>   // std::string
#include <utility>  // std::pair
#include <vector>   // std::vector

#include <opc/ua/node.h>                    // OpcUa::Node
#include <opc/ua/protocol/variant.h>        // OpcUa::Variant
#include <opc/ua/protocol/attribute_ids.h>  // OpcUa::AttributeId
#include <opc/ua/subscription.h>            // OpcUa::Subscription, OpcUa::SubscriptionHandler
#include <opencv2/calib3d.hpp>              // cv::Rodrigues(), cv::solvePnP()
#include <opencv2/core/mat.hpp>             // cv::Mat
#include <opencv2/core/types.hpp>           // cv::Point2d, cv::Point3d
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>

#include <cognex_demo/plc_interface.h>

// Anonymous namespace to prevent helper functions from 'leaking'
namespace
{
std::vector<cv::Point3d> toCV(const std::vector<geometry_msgs::Pose>& poses)
{
  std::vector<cv::Point3d> points;
  points.reserve(poses.size());
  for (const geometry_msgs::Pose& p : poses)
  {
    points.emplace_back(cv::Point3d(p.position.x, p.position.y, p.position.z));
  }
  return points;
}

std::vector<cv::Point2d> toCV(const std::vector<std::pair<float, float>>& uv)
{
  std::vector<cv::Point2d> points;
  points.reserve(uv.size());
  for (const std::pair<float, float>& p : uv)
  {
    points.emplace_back(cv::Point2d(static_cast<double>(p.first), static_cast<double>(p.second)));
  }
  return points;
}

geometry_msgs::PoseStamped toROS(const cv::Mat& rotation_vec, const cv::Mat& translation_vec)
{
  // Get the rotation part into a rotation matrix
  cv::Mat rotation_matrix = cv::Mat(3,3, CV_64F);
  cv::Rodrigues(rotation_vec, rotation_matrix);

  // Convert to tf2, then a quaternion
  tf2::Matrix3x3 tf_mat;
  tf_mat[0][0] = rotation_matrix.at<double>(0, 0);
  tf_mat[0][1] = rotation_matrix.at<double>(0, 1);
  tf_mat[0][2] = rotation_matrix.at<double>(0, 2);
  tf_mat[1][0] = rotation_matrix.at<double>(1, 0);
  tf_mat[1][1] = rotation_matrix.at<double>(1, 1);
  tf_mat[1][2] = rotation_matrix.at<double>(1, 2);
  tf_mat[2][0] = rotation_matrix.at<double>(2, 0);
  tf_mat[2][1] = rotation_matrix.at<double>(2, 1);
  tf_mat[2][2] = rotation_matrix.at<double>(2, 2);
  tf2::Quaternion q;
  tf_mat.getRotation(q);

  // Extract the values into a geometry_msgs::PoseStamped
  geometry_msgs::PoseStamped p;
  p.pose.position.x = translation_vec.at<double>(0);
  p.pose.position.y = translation_vec.at<double>(1);
  p.pose.position.z = translation_vec.at<double>(2);
  p.pose.orientation.w = q.getW();
  p.pose.orientation.x = q.getX();
  p.pose.orientation.y = q.getY();
  p.pose.orientation.z = q.getZ();

  return p;
}

tf2::Transform toTF2(const geometry_msgs::PoseStamped& pose)
{
  tf2::Vector3 v (
        pose.pose.position.x,
        pose.pose.position.y,
        pose.pose.position.z);
  tf2::Quaternion q (
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w);
  tf2::Transform tf (q, v);
  return tf;
}

geometry_msgs::PoseStamped reversePose(const geometry_msgs::PoseStamped& pose)
{
  tf2::Transform tf = toTF2(pose);
  tf = tf.inverse();
  tf2::Quaternion q = tf.getRotation();
  tf2::Vector3 v = tf.getOrigin();

  geometry_msgs::PoseStamped p;
  p.pose.position.x = v.getX();
  p.pose.position.y = v.getY();
  p.pose.position.z = v.getZ();
  p.pose.orientation.w = q.getW();
  p.pose.orientation.x = q.getX();
  p.pose.orientation.y = q.getY();
  p.pose.orientation.z = q.getZ();

  return p;
}

} // namespace anonymous

namespace demo
{

class LocationSolver
{
public:
  LocationSolver(ros::NodeHandle& nh, const std::string& camera_info_topic)
    : nh_ (nh)
    , sub_def (this)
  {
    // Load the camera intrinsics from the camera_info topic
    double fx, fy, cx, cy, k1, k2, k3, p1, p2;
    camera_matrix_ = cv::Mat(3, 3, CV_64F);
    const sensor_msgs::CameraInfo::ConstPtr& info_msg =
        ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_info_topic);
    if (info_msg == nullptr || info_msg->K[0] == 0.0)
    {
      ROS_ERROR_STREAM("Could not load camera info from topic: " << camera_info_topic);
    }
    else
    {
      k1 = info_msg->D[0];
      k2 = info_msg->D[1];
      p1 = info_msg->D[2];
      p2 = info_msg->D[3];
      k3 = info_msg->D[4];
      fx = info_msg->K[0];
      cx = info_msg->K[2];
      fy = info_msg->K[4];
      cy = info_msg->K[5];

      // Construct the intrinsics matrix
      camera_matrix_.at<double>(0, 0) = fx;
      camera_matrix_.at<double>(0, 1) = 0.0;
      camera_matrix_.at<double>(0, 2) = cx;
      camera_matrix_.at<double>(1, 0) = 0.0;
      camera_matrix_.at<double>(1, 1) = fy;
      camera_matrix_.at<double>(1, 2) = cy;
      camera_matrix_.at<double>(2, 0) = 0.0;
      camera_matrix_.at<double>(2, 1) = 0.0;
      camera_matrix_.at<double>(2, 2) = 1.0;
    }

    // Create a publisher to display the calculated target pose in RViz
    location_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/pose_relative_to_camera", 3);

    // Create a subscription handler
    sub = plci_.client.CreateSubscription(1, sub_def);

    // Make sure the subscription was created.
    if (sub != nullptr)
    {
      // For now, we are subscribing to the last component of the last position.  It would be wise to
      // instead subscribe to a bit whose intent is to signify 'all done writing'.  It could just be
      // an 8-bit integer that gets incremented when done writing u, v positions.
      OpcUa::Node opcua_node = plci_.client.GetNode("ns=3;s=\"DB_1\".\"feature_positions\"[19].\"v\"");
      subscription_handle = sub->SubscribeDataChange(opcua_node);
    }

    return;
  }

  ~LocationSolver()
  {
    // If we ever created the subscription, destroy it
    if (sub != nullptr)
    {
      sub->UnSubscribe(subscription_handle);
    }
    return;
  }

  // Define a class to define what happens when our subscription is triggered.  An object of this
  // class will be instantiated during construction of every LocationSolver.
  class SubscriptionCallbackDefinition : public OpcUa::SubscriptionHandler
  {
  public:
    SubscriptionCallbackDefinition(LocationSolver* ls)
      : ls_(ls)
    {
      return;
    }

    // DataChange callback subscription
    void DataChange(uint32_t /*handle*/,
                    const OpcUa::Node& /*node*/,
                    const OpcUa::Variant& /*value*/,
                    OpcUa::AttributeId /*attr*/) override
    {
      if (ls_ != nullptr)
      {

        // Pull UVs (pushed by FeatureDetectorNode to PLC data block)
        std::vector<std::pair<float, float>> uv_coords (20);
        ls_->plci_.readTag(node_ids::FEATURE_UV_PATH, uv_coords);

        // Pull poses of identifying features rel. to target, as defined in PLC
        std::vector<geometry_msgs::Pose> circle_poses (20);
        ls_->plci_.readTag(node_ids::FEATURE_TF_PATH, circle_poses);

        // Convert uv and pose to opencv types
        std::vector<cv::Point2d> uv_pts = toCV(uv_coords);
        std::vector<cv::Point3d> xyz_pts = toCV(circle_poses);

        // Solve the target pose
        cv::Mat distortion_coefficients;                  // leave as zero, see what happens
        cv::Mat rotation_vec = cv::Mat(3, 1, CV_64F);     // output parameter
        cv::Mat translation_vec = cv::Mat(3, 1, CV_64F);  // output parameter
        cv::solvePnP(
              xyz_pts,
              uv_pts,
              ls_->camera_matrix_,
              distortion_coefficients,
              rotation_vec,
              translation_vec);

        // Convert solution back to ROS types
        geometry_msgs::PoseStamped pose = toROS(rotation_vec, translation_vec);

        // Send it to the PLC
        ls_->plci_.writeTag(node_ids::TARGET_POSE, pose);

        // Send it to RViz
        pose = reversePose(pose);
        pose.header.frame_id = "plane";
        ls_->location_pub_.publish(pose);
      }

      return;
    }

  private:
    // Pointer to the parent LocationSolver
    LocationSolver* const ls_;
  };

private:
  ros::NodeHandle nh_;
  ros::Publisher location_pub_;
  cv::Mat camera_matrix_;

  PLCInterface plci_;
  SubscriptionCallbackDefinition sub_def;
  std::shared_ptr<OpcUa::Subscription> sub;
  uint32_t subscription_handle;

};  // class LocationSolver

} // namespace demo

int main(int argc, char **argv)
{
  ros::init(argc, argv, "location_solver_node");
  ros::NodeHandle nh;

  const std::string camera_info_topic = "/pylon_camera_node/camera_info";

  demo::LocationSolver ls(nh, camera_info_topic);

  ros::spin();
  return 0;
}
