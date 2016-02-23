#include "torch-tf.h"

TFIMP(tf::Quaternion *, Quaternion, new)()
{
  return new tf::Quaternion();
}

TFIMP(tf::Quaternion *, Quaternion, clone)(tf::Quaternion *self)
{
  return new tf::Quaternion(*self);
}

TFIMP(void, Quaternion, delete)(tf::Quaternion *self)
{
  delete self;
}

TFIMP(void, Quaternion, setIdentity)(tf::Quaternion *self)
{
  *self = tf::Quaternion::getIdentity();
}

TFIMP(void, Quaternion, setRotation_Tensor)(tf::Quaternion *self, THDoubleTensor *axis, double angle)
{
  tf::Vector3 v;
  copyTensorToVector3(axis, v);
  self->setRotation(v, angle);
}

TFIMP(void, Quaternion, setEuler)(tf::Quaternion *self, double yaw, double pitch, double roll)
{
  self->setEuler(yaw, pitch, roll);
}

TFIMP(void, Quaternion, getRPY)(tf::Quaternion *self, int solution_number, THDoubleTensor *result)
{
  double roll = 0, pitch = 0, yaw = 0;
  tf::Matrix3x3(*self).getRPY(roll, pitch, yaw, solution_number);
  copyVector3ToTensor(tf::Vector3(roll, pitch, yaw), result);
}

TFIMP(void, Quaternion, setRPY)(tf::Quaternion *self, double roll, double pitch, double yaw)
{
  self->setRPY(roll, pitch, yaw);
}

TFIMP(double, Quaternion, getAngle)(tf::Quaternion *self)
{
  return self->getAngle();
}

TFIMP(void, Quaternion, getAxis_Tensor)(tf::Quaternion *self, THDoubleTensor *axis)
{
  const tf::Vector3& a = self->getAxis();
  copyVector3ToTensor(a, axis);
}

TFIMP(void, Quaternion, inverse)(tf::Quaternion *self, tf::Quaternion *result)
{
  *result = self->inverse();
}

TFIMP(double, Quaternion, length2)(tf::Quaternion *self)
{
  return self->length2();
}

TFIMP(void, Quaternion, normalize)(tf::Quaternion *self)
{
  self->normalize();
}

TFIMP(double, Quaternion, angle)(tf::Quaternion *self, tf::Quaternion *other)
{
  return self->angle(*other);
}

TFIMP(double, Quaternion, angleShortestPath)(tf::Quaternion *self, tf::Quaternion *other)
{
  return self->angleShortestPath(*other);
}

TFIMP(void, Quaternion, add)(tf::Quaternion *self, tf::Quaternion *other, tf::Quaternion *result)
{
  *result = self->operator+(*other);
}

TFIMP(void, Quaternion, sub)(tf::Quaternion *self, tf::Quaternion *other, tf::Quaternion *result)
{
  *result = self->operator-(*other);
}

TFIMP(void, Quaternion, mul)(tf::Quaternion *self, tf::Quaternion *other, tf::Quaternion *result)
{
  if (result != self)
    *result = *self;
  result->operator*=(*other);
}

TFIMP(void, Quaternion, mul_scalar)(tf::Quaternion *self, double factor, tf::Quaternion *result)
{
  *result = self->operator*(factor);
}

TFIMP(void, Quaternion, div_scalar)(tf::Quaternion *self, double divisor, tf::Quaternion *result)
{
  *result = self->operator/(divisor);
}

TFIMP(double, Quaternion, dot)(tf::Quaternion *self, tf::Quaternion *other)
{
  return self->dot(*other);
}

TFIMP(void, Quaternion, slerp)(tf::Quaternion *self, tf::Quaternion *other, double t, tf::Quaternion *result)
{
  *result = self->slerp(*other, t);
}

TFIMP(void, Quaternion, viewTensor)(tf::Quaternion *self, THDoubleTensor *result)
{
  viewQuaternion(*self, result);
}
