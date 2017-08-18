#include "torch-ros.h"
#include <ros/master.h>

ROSIMP(bool, Master, execute)(
  const char *method,
  xamla::Variable *request,
  xamla::Variable *response,
  xamla::Variable *payload,
  bool wait_for_master) {

  XmlRpc::XmlRpcValue request_;
  VariableToXmlRpcValue(*request, request_);    // copy request

  XmlRpc::XmlRpcValue response_, payload_;
  bool result = ros::master::execute(method, request_, response_, payload_, wait_for_master);

  XmlRpcValueToVariable(response_, *response);  // copy response
  XmlRpcValueToVariable(payload_, *payload);
  return result;
}

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

ROSIMP(bool, Master, getTopics)(xamla::VariableTable_ptr *output) {
  if (!*output) {
    output->reset(new xamla::VariableTable());
  }

  xamla::VariableTable& output_table = **output;

  // call to maste api
  ros::master::V_TopicInfo topics;
  bool result = ros::master::getTopics(topics);

  // store result in table
  ros::master::V_TopicInfo::const_iterator i = topics.begin();
  for (; i != topics.end(); ++i) {
    const ros::master::TopicInfo& topic = *i;
    xamla::VariableTable_ptr t(new xamla::VariableTable());
    (*t)["topic"] = topic.name;
    (*t)["datatype"] = topic.datatype;
    output_table[topic.name] = t;
  }
  return result;
}

ROSIMP(bool, Master, getNodes)(std::vector<std::string> *output) {
  return ros::master::getNodes(*output);
}

ROSIMP(void, Master, setRetryTimeout)(int sec, int nsec) {
  ros::master::setRetryTimeout(ros::WallDuration(sec, nsec));
}
