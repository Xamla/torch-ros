#include "torch-ros.h"

ROSIMP(ros::Subscriber*, Subscriber, clone)(ros::Subscriber *self) {
  return new ros::Subscriber(*self);
}

ROSIMP(void, Subscriber, delete)(ros::Subscriber *self) {
  delete self;
}

ROSIMP(void, Subscriber, shutdown)(ros::Subscriber *self) {
  self->shutdown();
}

ROSIMP(const char *, Subscriber, getTopic)(ros::Subscriber *self) {
  return self->getTopic().c_str();
}

ROSIMP(int, Subscriber, getNumPublishers)(ros::Subscriber *self) {
  return static_cast<int>(self->getNumPublishers());
}
