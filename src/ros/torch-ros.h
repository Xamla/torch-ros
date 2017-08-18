#ifndef torch_ros_h
#define torch_ros_h

extern "C" {
#include <TH/TH.h>
}

#include <stdexcept>
#include <ros/ros.h>
#include "../std/variable.h"

#define ROSIMP(return_type, class_name, name) extern "C" return_type TH_CONCAT_4(ros_, class_name, _, name)

class RosWrapperException
  : public std::runtime_error {
public:
  RosWrapperException(const std::string& reason)
    : runtime_error(reason) {
  }
};


inline void VariableToXmlRpcValue(const xamla::Variable &src, XmlRpc::XmlRpcValue &dst) {
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
    case xamla::VariableType::Vector: {
      const xamla::VariableVector &v = *src.get_vector();
      dst.setSize(v.size());
      for (size_t i=0; i < v.size(); ++i) {
        VariableToXmlRpcValue(v[i], dst[i]);
      }
    } break;
    case xamla::VariableType::Table: {
      const xamla::VariableTable &t = *src.get_table();
      for (xamla::VariableTable::const_iterator i=t.begin(); i != t.end(); ++i) {
        const std::string& name = i->first;
        const xamla::Variable& value = i->second;
        VariableToXmlRpcValue(value, dst[name]);
      }
    } break;
  }
}

inline void XmlRpcValueToVariable(XmlRpc::XmlRpcValue &src, xamla::Variable &dst) {
  switch (src.getType()) {
    case XmlRpc::XmlRpcValue::TypeInvalid: dst.clear(); break;
    case XmlRpc::XmlRpcValue::TypeBoolean: dst.set_bool(src); break;
    case XmlRpc::XmlRpcValue::TypeInt: dst.set_int32(src); break;
    case XmlRpc::XmlRpcValue::TypeDouble: dst.set_float64(src); break;
    case XmlRpc::XmlRpcValue::TypeString: dst.set_string(src); break;
    case XmlRpc::XmlRpcValue::TypeArray: {
      xamla::VariableVector_ptr v(new xamla::VariableVector());
      for (int i = 0; i < src.size(); ++i) {
        xamla::Variable x;
        XmlRpcValueToVariable(src[i], x);
        v->push_back(x);
      }
      dst.set_vector(v);
    } break;
    case XmlRpc::XmlRpcValue::TypeStruct: {
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
#endif  // torch_ros_h
