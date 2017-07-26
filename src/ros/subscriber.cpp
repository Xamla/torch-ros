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

ROSIMP(void, Subscriber, getTopic)(ros::Subscriber *self, std::string *output) {
  *output = self->getTopic();
}

ROSIMP(int, Subscriber, getNumPublishers)(ros::Subscriber *self) {
  return static_cast<int>(self->getNumPublishers());
}
