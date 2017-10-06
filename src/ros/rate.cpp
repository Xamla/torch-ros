#include "torch-ros.h"

ROSIMP(ros::Rate*, Rate, new)(double frequence) {
  return new ros::Rate(frequence);
}

ROSIMP(void, Rate, delete)(ros::Rate *self) {
  delete self;
}

ROSIMP(ros::Rate*, Rate, clone)(ros::Rate *self) {
  return new ros::Rate(*self);
}

ROSIMP(void, Rate, reset)(ros::Rate *self) {
  self->reset();
}

ROSIMP(void, Rate, sleep)(ros::Rate *self) {
  self->sleep();
}

ROSIMP(void, Rate, expectedCycleTime)(ros::Rate *self, ros::Duration* output) {
  *output = self->expectedCycleTime();
}

ROSIMP(void, Rate, cycleTime)(ros::Rate *self, ros::Duration* output) {
  *output = self->cycleTime();
}
