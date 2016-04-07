#include "torch-ros.h"
#include <ros/callback_queue.h>

ROSIMP(ros::CallbackQueue *, CallbackQueue, new)(bool enabled) {
  return new ros::CallbackQueue(enabled);
}

ROSIMP(void, CallbackQueue, delete)(ros::CallbackQueue *self) {
  delete self;
}

ROSIMP(int, CallbackQueue, callOne)(ros::CallbackQueue *self, ros::Duration *timeout) {
  return static_cast<int>(self->callOne(timeout == NULL ? ros::WallDuration() : ros::WallDuration(timeout->sec, timeout->nsec)));
}

ROSIMP(void, CallbackQueue, callAvailable)(ros::CallbackQueue *self, ros::Duration *timeout) {
  self->callAvailable(timeout == NULL ? ros::WallDuration() : ros::WallDuration(timeout->sec, timeout->nsec));
}

ROSIMP(bool, CallbackQueue, isEmpty)(ros::CallbackQueue *self) {
  return self->isEmpty();
}

ROSIMP(void, CallbackQueue, clear)(ros::CallbackQueue *self) {
  self->clear();
}

ROSIMP(void, CallbackQueue, enable)(ros::CallbackQueue *self) {
  self->enable();
}

ROSIMP(void, CallbackQueue, disable)(ros::CallbackQueue *self) {
  self->disable();
}

ROSIMP(bool, CallbackQueue, isEnabled)(ros::CallbackQueue *self) {
  return self->isEnabled();
}
