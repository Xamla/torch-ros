#include "torch-ros.h"

ROSIMP(ros::AsyncSpinner*, AsyncSpinner, new)(uint32_t thread_count) {
  return new ros::AsyncSpinner(thread_count);
}

ROSIMP(void, AsyncSpinner, delete)(ros::AsyncSpinner *self) {
  delete self;
}

ROSIMP(bool, AsyncSpinner, canStart)(ros::AsyncSpinner *self) {
  return self->canStart();
}

ROSIMP(void, AsyncSpinner, start)(ros::AsyncSpinner *self) {
  self->start();
}

ROSIMP(void, AsyncSpinner, stop)(ros::AsyncSpinner *self) {
  self->stop();}
