#include "torch-ros.h"
#include <ros/callback_queue.h>

namespace xamla {

class WaitableCallbackQueue : public ros::CallbackQueue {
public:
  WaitableCallbackQueue(bool enabled = true)
   : ros::CallbackQueue(enabled) {}

  // only called from the single lua thread (TLS and pending calls not taken into account)
  bool waitCallAvailable(ros::WallDuration timeout) {
    boost::mutex::scoped_lock lock(mutex_);
    if (!enabled_)
      return false;

    if (callbacks_.empty() && !timeout.isZero())
      condition_.wait_for(lock, boost::chrono::nanoseconds(timeout.toNSec()));

    return !callbacks_.empty() && enabled_;
  }
};

} // namespace xamla


ROSIMP(xamla::WaitableCallbackQueue *, CallbackQueue, new)(bool enabled) {
  return new xamla::WaitableCallbackQueue(enabled);
}

ROSIMP(void, CallbackQueue, delete)(xamla::WaitableCallbackQueue *self) {
  delete self;
}

ROSIMP(int, CallbackQueue, callOne)(xamla::WaitableCallbackQueue *self, ros::Duration *timeout) {
  return static_cast<int>(self->callOne(timeout == NULL ? ros::WallDuration() : ros::WallDuration(timeout->sec, timeout->nsec)));
}

ROSIMP(void, CallbackQueue, callAvailable)(xamla::WaitableCallbackQueue *self, ros::Duration *timeout) {
  self->callAvailable(timeout == NULL ? ros::WallDuration() : ros::WallDuration(timeout->sec, timeout->nsec));
}

ROSIMP(bool, CallbackQueue, waitCallAvailable)(xamla::WaitableCallbackQueue *self, ros::Duration *timeout) {
  return self->waitCallAvailable(timeout == NULL ? ros::WallDuration() : ros::WallDuration(timeout->sec, timeout->nsec));
}

ROSIMP(bool, CallbackQueue, isEmpty)(xamla::WaitableCallbackQueue *self) {
  return self->isEmpty();
}

ROSIMP(void, CallbackQueue, clear)(xamla::WaitableCallbackQueue *self) {
  self->clear();
}

ROSIMP(void, CallbackQueue, enable)(xamla::WaitableCallbackQueue *self) {
  self->enable();
}

ROSIMP(void, CallbackQueue, disable)(xamla::WaitableCallbackQueue *self) {
  self->disable();
}

ROSIMP(bool, CallbackQueue, isEnabled)(xamla::WaitableCallbackQueue *self) {
  return self->isEnabled();
}
