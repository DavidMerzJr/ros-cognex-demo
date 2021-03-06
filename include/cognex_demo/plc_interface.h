#ifndef PLC_INTERFACE_H
#define PLC_INTERFACE_H

#include <string>   // std::string
#include <utility>  // std::pair
#include <vector>   // std::vector

#include <opc/ua/client/client.h>     // OpcUa::UaClient
#include <opc/ua/protocol/variant.h>  // OpcUa::Variant

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

namespace demo
{

const static std::string OPCUA_ENDPOINT = "opc.tcp://192.168.10.8:4840";

/*
// Example subscription.

// include the header
#include <opc/ua/subscription.h>

// define a class
class SubscriptionCallbackDefinition : public OpcUa::SubscriptionHandler
{
  void DataChange(uint32_t handle,
                  const OpcUa::Node& node,
                  const OpcUa::Variant& value,
                  OpcUa::AttributeId attr) override
  {
    // Do code here based on the situation at hand.
  }
};

// On instantiation
SubscriptionCallbackDefinition sub_def;
int i = 1; // Not sure what this does yet.
std::shared_ptr<OpcUa::Subscription> sub = client.CreateSubscription(i, sub_def);

// To subscribe
std::string node_id = "NAME"; // obviously this changes from case to case.
OpcUa::Node opcua_node = client_.GetNode(node_id);
uint32_t subscription_handle = sub->SubscribeDataChange(opcua_node);

// To unsubscribe on destruct
sub->UnSubscribe(subscription_handle);
*/

namespace node_ids
{

// ns=3;s="DB_1"."target_pose"."r"
const static std::string TARGET_POSE = "ns=3;s=\"DB_1\".\"target_pose\"";

// ns=3;s="DB_1"."feature_positions"[0]."u"
const static std::string FEATURE_UV_PATH = "ns=3;s=\"DB_1\".\"feature_positions\"[";

// ns=3;s="DB_1"."feature_transforms_array"[0]."x"
const static std::string FEATURE_TF_PATH = "ns=3;s=\"DB_1\".\"feature_transforms_array\"[";

const static std::string OPCUA_TEST_NODEID = "ns=3;s=\"DB_1\".\"target_pose\".\"x\"";

} //namespace node_ids

class PLCInterface
{
public:
  PLCInterface();
  ~PLCInterface();

  // reading tags
  bool readTag(const std::string& node_id, std::vector<std::pair<float, float>>& value);
  bool readTag(const std::string& node_id, double& value);
  bool readTag(const std::string& node_id, float& value);
  bool readTag(const std::string& node_id, std::vector<geometry_msgs::Pose>& value);

  // writing tags:
  bool writeTag(const std::string& node_id, const std::vector<std::pair<float, float>> value);
  bool writeTag(const std::string& node_id, const geometry_msgs::PoseStamped& value);
  bool writeTag(const std::string& node_id, float& value);

  OpcUa::UaClient client;

private:
  std::string endpoint_;
  bool connect(const std::string& endpoint);
  bool disconnect();

  bool readTag(const std::string& node_id, OpcUa::Variant& value);
  bool writeTag(const std::string& node_id, const OpcUa::Variant& value);

  std::string generateArrayNodeID(
      const std::string& node_id,
      const uint index,
      const std::string& dim);
  std::string generateDimNodeID(
      const std::string& node_id,
      const std::string& dim);

};  // class PLCInterface

} // namespace demo

#endif // PLC_INTERFACE_H
