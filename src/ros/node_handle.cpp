#include "torch-ros.h"

ROSIMP(ros::NodeHandle*, NodeHandle, new)() {
  return new ros::NodeHandle();
}

ROSIMP(void, NodeHandle, delete)(ros::NodeHandle *self) {
  delete self;
}
