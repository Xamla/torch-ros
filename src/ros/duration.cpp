#include "torch-ros.h"

ROSIMP(ros::Duration*, Duration, new)() {
  return new ros::Duration();
}

ROSIMP(void, Duration, delete)(ros::Duration *self) {
  delete self;
}

ROSIMP(ros::Duration*, Duration, clone)(ros::Duration *self) {
  return new ros::Duration(*self);
}

ROSIMP(void, Duration, set)(ros::Duration *self, int sec, int nsec) {
  self->sec = sec;
  self->nsec = nsec;
}

ROSIMP(void, Duration, assign)(ros::Duration *self, ros::Duration *other) {
  *self = *other;
}

ROSIMP(int, Duration, get_sec)(ros::Duration *self) {
  return self->sec;
}

ROSIMP(void, Duration, set_sec)(ros::Duration *self, int sec) {
  self->sec = sec;
}

ROSIMP(int, Duration, get_nsec)(ros::Duration *self) {
  return self->nsec;
}

ROSIMP(void, Duration, set_nsec)(ros::Duration *self, int nsec) {
  self->nsec = nsec;
}

ROSIMP(void, Duration, add)(ros::Duration *self, ros::Duration *other, ros::Duration *result) {
  *result = *self + *other;
}

ROSIMP(void, Duration, sub)(ros::Duration *self, ros::Duration *other, ros::Duration *result) {
  *result = *self - *other;
}

ROSIMP(void, Duration, mul)(ros::Duration *self, double scale, ros::Duration *result) {
  *result = *self * scale;
}

ROSIMP(bool, Duration, eq)(ros::Duration *self, ros::Duration *other) {
  return *self == *other;
}

ROSIMP(bool, Duration, lt)(ros::Duration *self, ros::Duration *other) {
  return *self < *other;
}

ROSIMP(double, Duration, toSec)(ros::Duration *self) {
  return self->toSec();
}

ROSIMP(void, Duration, fromSec)(ros::Duration *self, double t) {
  self->fromSec(t);
}

ROSIMP(bool, Duration, isZero)(ros::Duration *self) {
  return self->isZero();
}

ROSIMP(void, Duration, sleep)(ros::Duration *self) {
  self->sleep();
}
