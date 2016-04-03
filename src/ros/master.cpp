#include "torch-ros.h"
#include <ros/master.h>
#include "../std/variable.h"

void VariableToXmlRpcValue(const xamla::Variable &src, XmlRpc::XmlRpcValue &dst) {
  switch (src.get_type()) {
    case xamla::VariableType::Void: dst.clear(); break;
    case xamla::VariableType::Bool: dst = XmlRpc::XmlRpcValue(src.get_bool()); break;
    case xamla::VariableType::Int8: dst = XmlRpc::XmlRpcValue(static_cast<int>(src.get_int8())); break;
    case xamla::VariableType::Int16: dst = XmlRpc::XmlRpcValue(static_cast<int>(src.get_int16())); break;
    case xamla::VariableType::Int32: dst = XmlRpc::XmlRpcValue(static_cast<int>(src.get_int32())); break;
    case xamla::VariableType::Int64: dst = XmlRpc::XmlRpcValue(static_cast<int>(src.get_int64())); break;
    case xamla::VariableType::UInt8: dst = XmlRpc::XmlRpcValue(static_cast<int>(src.get_uint8())); break;
    case xamla::VariableType::UInt16: dst = XmlRpc::XmlRpcValue(static_cast<int>(src.get_uint16())); break;
    case xamla::VariableType::UInt32: dst = XmlRpc::XmlRpcValue(static_cast<int>(src.get_uint32())); break;
    case xamla::VariableType::UInt64: dst = XmlRpc::XmlRpcValue(static_cast<int>(src.get_uint64())); break;
    case xamla::VariableType::Float32: dst = XmlRpc::XmlRpcValue(src.get_float32()); break;
    case xamla::VariableType::Float64: dst = XmlRpc::XmlRpcValue(src.get_float64()); break;
    case xamla::VariableType::String: dst = XmlRpc::XmlRpcValue(src.get_string()); break;

    case xamla::VariableType::Vector:
      break;
    case xamla::VariableType::Table:
      break;
  }
}

void XmlRpcValueToVariable(XmlRpc::XmlRpcValue &src, xamla::Variable &dst) {
  switch (src.getType()) {
    case XmlRpc::XmlRpcValue::TypeInvalid: dst.clear(); break;
    case XmlRpc::XmlRpcValue::TypeBoolean: dst.set_bool(src); break;
    case XmlRpc::XmlRpcValue::TypeInt: dst.set_int32(src); break;
    case XmlRpc::XmlRpcValue::TypeDouble: dst.set_float64(src); break;
    case XmlRpc::XmlRpcValue::TypeString: dst.set_string(src); break;
    case XmlRpc::XmlRpcValue::TypeArray:
    {
      xamla::VariableVector_ptr v(new xamla::VariableVector());
      for (int i = 0; i < src.size(); ++i) {
        xamla::Variable x;
        XmlRpcValueToVariable(src[i], x);
        v->push_back(x);
      }
      dst.set_vector(v);
    } break;
    case XmlRpc::XmlRpcValue::TypeStruct:
    {
      xamla::VariableTable_ptr t(new xamla::VariableTable());

      for (XmlRpc::XmlRpcValue::iterator i = src.begin(); i != src.end(); ++i) {
        const std::string &name = i->first;
        XmlRpc::XmlRpcValue &value = i->second;
        xamla::Variable v;
        XmlRpcValueToVariable(value, v);
        (*t)[name] = v;
      }
      dst.set_table(t);
    } break;
    case XmlRpc::XmlRpcValue::TypeDateTime:
    case XmlRpc::XmlRpcValue::TypeBase64:
      break;
  }
}

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
