#include "torch-ros.h"
#include <ros/master.h>
#include "../std/variable.h"

//ROSCPP_DECL bool execute(const std::string& method, const XmlRpc::XmlRpcValue& request, XmlRpc::XmlRpcValue& response, XmlRpc::XmlRpcValue& payload, bool wait_for_master);

ROSIMP(const char*, Master, getHost)() {
  return ros::master::getHost().c_str();
}

ROSIMP(int, Master, getPort)() {
  return static_cast<int>(ros::master::getPort());
}

ROSIMP(const char*, Master, getURI)() {
  return ros::master::getURI().c_str();
}

ROSIMP(bool, Master, check)() {
  return ros::master::check();
}

ROSIMP(bool, Master, getTopics)(xamla::VariableVector_ptr *output) {
  xamla::VariableVector& v = **output;
  v.clear();

  ros::master::V_TopicInfo topics;
  bool result = ros::master::getTopics(topics);
  ros::master::V_TopicInfo::const_iterator i = topics.begin();
  for (; i != topics.end(); ++i) {
    const ros::master::TopicInfo& topic = *i;
    xamla::VariableTable_ptr t(new xamla::VariableTable());
    (*t)["name"] = topic.name;
    (*t)["datatype"] = topic.datatype;
    v.push_back(t);
  }
  return result;
}

ROSIMP(bool, Master, getNodes)(std::vector<std::string> *output) {
  return ros::master::getNodes(*output);
}

ROSIMP(void, Master, setRetryTimeout)(int sec, int nsec) {
  ros::master::setRetryTimeout(ros::WallDuration(sec, nsec));
}
