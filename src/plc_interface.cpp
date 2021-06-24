#include <cognex_demo/plc_interface.h>

namespace demo
{

PLCInterface::PLCInterface()
{
  return;
};
PLCInterface::~PLCInterface()
{
  return;
};

bool PLCInterface::init(const std::string& endpoint = OPCUA_ENDPOINT)
{
  bool success = true;
  success &= connect(endpoint);
  return success;
};
bool PLCInterface::connect(const std::string& endpoint)
{
  bool success = true;
  try {
    client.Connect(endpoint);
    success=true;
  }  catch (const std::exception& ex) {
    //ROS_ERROR_STREAM("Error connecting to opcua server: " << ex.what());
    success = false;
  } catch (...){
    //ROS_ERROR_STREAM("Error connecting to opcua server: Unknown Exception." );
    success = false;
  }
  return success;
};
bool PLCInterface::disconnect()
{
  bool success = true;
  try {
    client.Disconnect();
    success=true;
  }  catch (const std::exception& ex) {
    //ROS_ERROR_STREAM("Error disconnecting from opcua server: " << ex.what());
    success = false;
  } catch (...){
    //ROS_ERROR_STREAM("Error disconnecting from opcua server: Unknown Exception." );
    success = false;
  }
  return success;
};


bool PLCInterface::readTag(const std::string& node_id, std::vector<std::pair<float, float>>& value)
{
  PLCInterface::readTag(node_id, value);
}

//bool PLCInterface::readTag(const std::string& node_id, std::vector<std::pair<float, float>>& value)
//{
//  //Code
//  bool success = true;
//  float out;
//  //  const std::vector<std::pair<float, float>> out;
//  const std::string u_id = node_id+".\"u\"";
//  const std::string v_id = node_id+".\"v\"";
//  if(readTag(u_id, out) )
//  {
//    value[1] = static_cast<float>out;
//  }
//  if(readTag(v_id, out) )
//  {
//    value[2] = static_cast<float>out;
//  }

//  return success;
//}

//bool PLCInterface::writeTag(const std::string& node_id, const std::vector<std::pair<int, int>> value);
//bool PLCInterface::writeTag(const std::string& node_id, const geometry_msgs::PoseStamped& value);
//bool PLCInterface::writeTag(const std::string& node_id, const OpcUa::Variant& value);

bool PLCInterface::readTag(const std::string& node_id, OpcUa::Variant& value)
{
  bool success = false;
  try {
    OpcUa::Node node = client.GetNode(node_id);
    value = static_cast<float>(node.GetValue());
    success = true;
  } catch (const std::exception& ex) {
    //ROS_ERROR_STREAM("Error reading opcua nodeID'" << node_id <<"' Error: " <<ex.what());
    success = false;
  } catch (...){
    //ROS_ERROR_STREAM("Error reading opcua nodeID'" << node_id <<"'. Unknown Exception." );
    success = false;
  }
  return success;
}

int main()
{
  PLCInterface plc;
  if(plc.init())
  {
    std::cout<<"OPCUA server connected.";

  }else{
    std::cout<<"OPCUA server connection failed! :(";
        //  return 1;
  }
  float testNumber = 5.5f;
  if(plc.readTag(demo::node_ids::OPCUA_TEST_NODEID, testNumber)){
     std::cout<<"Pulled value from PLC: " <<testNumber;
  }else{
    std::cout<<"Failed to get value! Test value is still 5.5: "<<testNumber;
  }
  return 0;
}


} // namespace demo
