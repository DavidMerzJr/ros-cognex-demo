#include <cognex_demo/plc_interface.h>

namespace demo
{

PLCInterface::PLCInterface();
PLCInterface::~PLCInterface();

bool PLCInterface::init(const std::string& endpoint = OPCUA_ENDPOINT);
bool PLCInterface::readTag(const std::string& node_id, std::vector<std::pair<int, int>>& value);
bool PLCInterface::readTag(const std::string& node_id, std::vector<geometry_msgs::Pose>& value);
bool PLCInterface::writeTag(const std::string& node_id, const std::vector<std::pair<int, int>> value);
bool PLCInterface::writeTag(const std::string& node_id, const geometry_msgs::PoseStamped& value);

bool PLCInterface::connect(const std::string& endpoint);
bool PLCInterface::disconnect();

bool PLCInterface::readTag(const std::string& node_id, OpcUa::Variant& value);
bool PLCInterface::writeTag(const std::string& node_id, const OpcUa::Variant& value);

} // namespace demo
