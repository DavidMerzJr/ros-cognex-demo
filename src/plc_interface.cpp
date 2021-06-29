#include <cognex_demo/plc_interface.h>

#include <exception>  // std::exception

#include <ros/console.h>

namespace demo
{

PLCInterface::PLCInterface()
{
  connect(OPCUA_ENDPOINT);
  return;
}

PLCInterface::~PLCInterface()
{
  disconnect();
  return;
}

bool PLCInterface::connect(const std::string& endpoint = OPCUA_ENDPOINT)
{
  bool success = true;
  try
  {
    ROS_DEBUG_STREAM("Begin PLC OPCUA server connect attempt.");
    client.Connect(endpoint);
    success = true;
  }
  catch (const std::exception& ex)
  {
    ROS_ERROR_STREAM("Error connecting to opcua server: " << ex.what()  );
    success = false;
  }
  catch (...)
  {
    ROS_ERROR_STREAM("Error connecting to opcua server: Unknown Exception." );
    success = false;
  }

  if (success)
  {
    ROS_DEBUG_STREAM("PLC OPCUA server connected!");
  }
  return success;
}

bool PLCInterface::disconnect()
{
  bool success = true;
  try
  {
    client.Disconnect();
    success = true;
  }
  catch (const std::exception& ex)
  {
    ROS_ERROR_STREAM("Error disconnecting from opcua server: "<< ex.what());
    success = false;
  }
  catch (...)
  {
    ROS_ERROR_STREAM("Error disconnecting from opcua server: Unknown Exception." );
    success = false;
  }
  return success;
}

std::string PLCInterface::generateArrayNodeID(const std::string& node_id, const uint index, const std::string& dim)
{
  return node_id + std::to_string(index) + "].\"" + dim + "\"";
}

std::string PLCInterface::generateDimNodeID(const std::string& node_id, const std::string& dim)
{
  return node_id + ".\"" + dim + "\"";
}

bool PLCInterface::readTag(const std::string& node_id, std::vector<std::pair<float, float>>& value)
{
  //Reads the (u,v) coordinate positions of features from PLC to a list of pairs.
  bool success = true;

  for (uint i = 0; i < value.size() && success; ++i)
  {
    if (!readTag(generateArrayNodeID(node_id, i, "u"), value[i].first))
    {
      success = false;
    }
    if (!readTag(generateArrayNodeID(node_id, i, "v"), value[i].second))
    {
      success = false;
    }
  }
  return success;
}

bool PLCInterface::readTag(const std::string& node_id, float& value)
{
  bool success = true;
  OpcUa::Variant opcua_value;
  if (!readTag(node_id, opcua_value))
  {
    return false;
  }
  if (static_cast<int>(opcua_value.Type()) == 10) //float32
  {
    value = static_cast<float>(opcua_value);
  }
  else if (static_cast<int>(opcua_value.Type()) == 11)  //float64
  {
    value = static_cast<float>(opcua_value);
  }
  else
  {
    success = false;
    ROS_ERROR_STREAM("Error reading opcua nodeID'" + node_id + "'. Node holds wrong data type.");
  }

  return success;
}

bool PLCInterface::readTag(const std::string& node_id, double& value)
{
  float fvalue = 0.0;
  bool success = readTag(node_id, fvalue);
  value = static_cast<double>(fvalue);
  return success;
}

bool PLCInterface::readTag(const std::string& node_id, std::vector<geometry_msgs::Pose>& value)
{
  bool success = true;

  for (uint i = 0; i < value.size() && success; ++i)
  {
    success &=
        PLCInterface::readTag(generateArrayNodeID(node_id, i, "x"), value[i].position.x)
        && PLCInterface::readTag(generateArrayNodeID(node_id, i, "y"), value[i].position.y)
        && PLCInterface::readTag(generateArrayNodeID(node_id, i, "z"), value[i].position.z)
        && PLCInterface::readTag(generateArrayNodeID(node_id, i, "rw"), value[i].orientation.w)
        && PLCInterface::readTag(generateArrayNodeID(node_id, i, "rx"), value[i].orientation.x)
        && PLCInterface::readTag(generateArrayNodeID(node_id, i, "ry"), value[i].orientation.y)
        && PLCInterface::readTag(generateArrayNodeID(node_id, i, "rz"), value[i].orientation.z);

    if (!success)
    {
      ROS_ERROR_STREAM("Failed to Read Pose at index "<< i );
    }
  }

  return success;
}

bool PLCInterface::readTag(const std::string& node_id, OpcUa::Variant& value)
{
  bool success = false;
  try
  {
    OpcUa::Node node = client.GetNode(node_id);
    value = static_cast<float>(node.GetValue());
    success = true;
  }
  catch (const std::exception& ex)
  {
    ROS_ERROR_STREAM("Error reading opcua nodeID'"  +  node_id  + "' Error: "  + ex.what());
    success = false;
  }
  catch (...)
  {
    ROS_ERROR_STREAM("Error reading opcua nodeID'"  +  node_id  + "'. Unknown Exception." );
    success = false;
  }
  return success;
}

bool PLCInterface::writeTag(const std::string& node_id, const std::vector<std::pair<float, float>> value)
{
  //writes the (u,v) coordinate positions of features from PLC to a list of pairs.
  bool success = true;

  for(uint i = 0; i < value.size(); i++) {
    if(!PLCInterface::writeTag(generateArrayNodeID(node_id, i, "u"), value[i].first))
    {
      success = false;
    }
    if(!PLCInterface::writeTag(generateArrayNodeID(node_id, i, "v"), value[i].second))
    {
      success = false;
    }
  }
  return success;
}

bool PLCInterface::writeTag(const std::string& node_id, const geometry_msgs::PoseStamped& value)
{
  bool success = true;

  // Value contains a header with system time but that is not being used here.

  float statx = static_cast<float>(value.pose.position.x);
  float staty = static_cast<float>(value.pose.position.y);
  float statz = static_cast<float>(value.pose.position.z);

  float statrw = static_cast<float>(value.pose.orientation.w);
  float statrx = static_cast<float>(value.pose.orientation.x);
  float statry = static_cast<float>(value.pose.orientation.y);
  float statrz = static_cast<float>(value.pose.orientation.z);

  success &=
      PLCInterface::writeTag(generateDimNodeID(node_id, "x"), statx)
      && PLCInterface::writeTag(generateDimNodeID(node_id, "y"), staty)
      && PLCInterface::writeTag(generateDimNodeID(node_id, "z"), statz)
      && PLCInterface::writeTag(generateDimNodeID(node_id, "rw"), statrw)
      && PLCInterface::writeTag(generateDimNodeID(node_id, "rx"), statrx)
      && PLCInterface::writeTag(generateDimNodeID(node_id, "ry"), statry)
      && PLCInterface::writeTag(generateDimNodeID(node_id, "rz"), statrz);
  if (!success)
  {
    ROS_ERROR_STREAM("Failed to write target pose!" );
  }

  return success;
}

bool PLCInterface::writeTag(const std::string& node_id, float& value)
{
  OpcUa::Variant var = value;
  return writeTag(node_id, var);
}

bool PLCInterface::writeTag(const std::string& node_id, const OpcUa::Variant& value)
{
  bool success = false;

  if (value.IsNul())
  {
    ROS_ERROR_STREAM("Cannot write null value.");
    return false;
  }
  else
  {
    try
    {
      OpcUa::Node node = client.GetNode(node_id);
      node.SetValue(value); // = static_cast<float>(node.GetValue());
      success = true;
    }
    catch (const std::exception& ex)
    {
      ROS_ERROR_STREAM("Error pulling node value: '" + node_id + "' error: " + ex.what());
    }
    catch(...)
    {
      ROS_ERROR_STREAM("Error pulling node value: '" + node_id + "', unknown error. ");
    }
  }
  return success;
}

} // namespace demo

/*  This file defines a class, and should be built as a library
int main()
{
  demo::PLCInterface plc;
  plc.echo("PLC instantiated.\n");

  // test some reading

  //* //test read/write UVs
  plc.echo("\nAttempting to pull UV values from "  +  plc.generateArrayNodeID(demo::node_ids::FEATURE_UV_PATH, 0, "u"));
  std::vector<std::pair<float, float>> uvs = {std::make_pair(1.1, 1.1), std::make_pair(2.2, 2.2),
                                              std::make_pair(3.3,3.3), std::make_pair(4.4,4.4)};
  std::vector<std::pair<float, float>> newuvs = {std::make_pair(1.1, 1.1), std::make_pair(2.2, 2.2),
                                              std::make_pair(3.3,3.3), std::make_pair(4.4,4.4)};
  if(plc.readTag(demo::node_ids::FEATURE_UV_PATH, uvs)){
    plc.echo("\nPulled values from PLC. \nOne: {"  +  std::to_string(uvs[0].first)  + ", "  + std::to_string(uvs[0].second)  +
               "}\n and two: {"  +  std::to_string(uvs[1].first)  + ", "  + std::to_string(uvs[1].second)  + "}"
               "}\n and three: {"  +  std::to_string(uvs[2].first)  + ", "  + std::to_string(uvs[2].second)  + "}"
               "}\n and four: {"  +  std::to_string(uvs[3].first)  + ", "  + std::to_string(uvs[3].second)  + "}");
  }else{
    plc.echo("\nFailed to get values!");
  }
  if(plc.writeTag(demo::node_ids::FEATURE_UV_PATH, newuvs))
  {
    plc.echo("\nSuccessfully wrote new values for UVs! \nOne: {"  +  std::to_string(newuvs[0].first)  + ", "  + std::to_string(newuvs[0].second)  +
        "}\n and two: {"  +  std::to_string(newuvs[1].first)  + ", "  + std::to_string(newuvs[1].second)  + "}"
        "}\n and three: {"  +  std::to_string(newuvs[2].first)  + ", "  + std::to_string(newuvs[2].second)  + "}"
        "}\n and four: {"  +  std::to_string(newuvs[3].first)  + ", "  + std::to_string(newuvs[3].second)  + "}");
  }
  if(plc.writeTag(demo::node_ids::FEATURE_UV_PATH, uvs))
  {
    plc.echo("\nWrote old values back: \nOne: {"  +  std::to_string(uvs[0].first)  + ", "  + std::to_string(uvs[0].second)  +
               "}\n and two: {"  +  std::to_string(uvs[1].first)  + ", "  + std::to_string(uvs[1].second)  + "}"
               "}\n and three: {"  +  std::to_string(uvs[2].first)  + ", "  + std::to_string(uvs[2].second)  + "}"
               "}\n and four: {"  +  std::to_string(uvs[3].first)  + ", "  + std::to_string(uvs[3].second)  + "}");
  }

  //test some writing:
  /*plc.echo("\n Test writing to OPCUA node: \n " );
  float oldVal = 0.0f;
  float val = 2.5f;
  if(plc.readTag(demo::node_ids::OPCUA_TEST_NODEID, oldVal))
  {
    plc.echo("\nSuccessfully took down old test value: " + std::to_string(oldVal));
    if(plc.writeTag(demo::node_ids::OPCUA_TEST_NODEID, val))
    {
      plc.echo("\nSuccessfully wrote new value! " );
      plc.readTag(demo::node_ids::OPCUA_TEST_NODEID, val);
      plc.echo("\nNew value is: " + std::to_string(val));
      if(plc.writeTag(demo::node_ids::OPCUA_TEST_NODEID, oldVal))
      {
        plc.echo("\nWrote original value back. \n " );

      }
    }
  }

  return 0;
}
*/
