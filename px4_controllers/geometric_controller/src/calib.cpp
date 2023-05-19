#include <ros/package.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h> // Include the YAML-CPP library
#include <fstream>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "calib_node");
  ros::NodeHandle nh("~"); // Use a private node handle for parameters

  // Define your parameters
  double param1 = 1.23;
  int param2 = 42;
  std::string param3 = "hello";

  // Save the parameters to a YAML file
  std::string yamlPath = ros::package::getPath("geometric_controller") + "/cfg/gps_calib.yaml";
  YAML::Emitter emitter;
  emitter << YAML::BeginMap;
  emitter << YAML::Key << "param1" << YAML::Value << param1;
  emitter << YAML::Key << "param2" << YAML::Value << param2;
  emitter << YAML::Key << "param3" << YAML::Value << param3;
  emitter << YAML::EndMap;

  std::ofstream file(yamlPath,std::ofstream::out | std::ofstream::trunc);
  file << emitter.c_str();
  file.close();

  return 0;
}
