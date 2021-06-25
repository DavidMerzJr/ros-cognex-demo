#ifndef FEATURE_DETECTOR_NODE_H
#define FEATURE_DETECTOR_NODE_H

#include <string>   // std::string
#include <utility>  // std::pair
#include <vector>   // std::vector

#include <opc/ua/client/client.h>
#include <opc/ua/subscription.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <target_finder/temp_file.h>

namespace feature_detector_node
{
demo::PLCInterface plc_interface_;
}// feature_detector_node



#endif // FEATURE_DETECTOR_NODE_H
