#include "torch-ros.h"

ROSIMP(ros::Publisher*, Publisher, clone)(ros::Publisher *self) {
  return new ros::Publisher(*self);
}

ROSIMP(void, Publisher, delete)(ros::Publisher *self) {
  delete self;
}

ROSIMP(void, Publisher, shutdown)(ros::Publisher *self) {
  self->shutdown();
}

ROSIMP(const char *, Publisher, getTopic)(ros::Publisher *self) {
  return self->getTopic().c_str();
}

ROSIMP(int, Publisher, getNumSubscribers)(ros::Publisher *self) {
  return static_cast<int>(self->getNumSubscribers());
}

ROSIMP(bool, Publisher, isLatched)(ros::Publisher *self) {
  return self->isLatched();
}
