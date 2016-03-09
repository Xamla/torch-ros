#ifndef torch_ros_h
#define torch_ros_h

extern "C" {
#include <TH/TH.h>
}

#include <stdexcept>
#include <ros/ros.h>

#define ROSIMP(return_type, class_name, name) extern "C" return_type TH_CONCAT_4(ros_, class_name, _, name)

class RosWrapperException
  : public std::runtime_error {
public:
  RosWrapperException(const std::string& reason)
    : runtime_error(reason) {
  }
};

class NotImplementedException
  : public std::logic_error {
public:
  NotImplementedException()
    : logic_error("Function not implemented.") {
  }

  NotImplementedException(const std::string &msg)
    : logic_error(msg) {
  }
};

#endif  // torch_ros_h
