#ifndef torch_ros_h
#define torch_ros_h

extern "C" {
#include <TH/TH.h>
}

#include <stdexcept>
#include <ros/ros.h>

#define ROSIMP(return_type, class_name, name) extern "C" return_type TH_CONCAT_4(ros_, class_name, _, name)

class RosWrapperException
  : public std::runtime_error
{
public:
  RosWrapperException(const std::string& reason)
    : runtime_error(reason)
  {
  }
};

#endif  // torch_ros_h
