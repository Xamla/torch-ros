#include "torch-ros.h"

ROSIMP(ros::Time*, Time, new)() {
  return new ros::Time();
}

ROSIMP(void, Time, delete)(ros::Time *self) {
  delete self;
}

ROSIMP(ros::Time*, Time, clone)(ros::Time *self) {
  return new ros::Time(*self);
}

ROSIMP(bool, Time, isZero)(ros::Time *self) {
  return self->isZero();
}

ROSIMP(void, Time, fromSec)(ros::Time *self, double t) {
  self->fromSec(t);
}

ROSIMP(double, Time, toSec)(ros::Time *self) {
  return self->toSec();
}

ROSIMP(void, Time, set)(ros::Time *self, unsigned int sec, unsigned int nsec) {
  self->sec = sec;
  self->nsec = nsec;
}

ROSIMP(void, Time, assign)(ros::Time *self, ros::Time *other) {
  *self = *other;
}

ROSIMP(int, Time, get_sec)(ros::Time *self) {
  return static_cast<int>(self->sec);
}

ROSIMP(void, Time, set_sec)(ros::Time *self, unsigned int sec) {
  self->sec = sec;
}

ROSIMP(int, Time, get_nsec)(ros::Time *self) {
  return static_cast<int>(self->nsec);
}

ROSIMP(void, Time, set_nesc)(ros::Time *self, unsigned int nsec) {
  self->nsec = nsec;
}

ROSIMP(bool, Time, lt)(ros::Time *self, ros::Time *other) {
  return self->operator<(*other);
}

ROSIMP(bool, Time, eq)(ros::Time *self, ros::Time *other) {
  return self->operator==(*other);
}

ROSIMP(void, Time, add_Duration)(ros::Time *self, ros::Duration *duration, ros::Time *result) {
  *result = self->operator+(*duration);
}

ROSIMP(void, Time, sub)(ros::Time *self, ros::Time *other, ros::Duration *result) {
  *result = self->operator-(*other);
}

ROSIMP(void, Time, sub_Duration)(ros::Time *self, ros::Duration *duration, ros::Time *result) {
  *result = self->operator-(*duration);
}

// static members

ROSIMP(void, Time, sleepUntil)(ros::Time *end) {
  ros::Time::sleepUntil(*end);
}

ROSIMP(void, Time, getNow)(ros::Time *result) {
  *result = ros::Time::now();
}

ROSIMP(void, Time, setNow)(ros::Time* now) {
  ros::Time::setNow(*now);
}
    
ROSIMP(void, Time, waitForValid)() {
  ros::Time::waitForValid();
}

ROSIMP(void, Time, init)() {
  ros::Time::init();
}

ROSIMP(void, Time, shutdown)() {
  ros::Time::shutdown();
}

ROSIMP(bool, Time, useSystemTime)() {
  return ros::Time::useSystemTime();
}

ROSIMP(bool, Time, isSimTime)() {
  return ros::Time::isSimTime();
}

ROSIMP(bool, Time, isSystemTime)() {
  return ros::Time::isSystemTime();
}

ROSIMP(bool, Time, isValid)() {
  return ros::Time::isValid();
}
