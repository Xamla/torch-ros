#include "torch-tf.h"

TFIMP(tf::Transform *, Transform, new)()
{
  return new tf::Transform();
}

TFIMP(tf::Transform *, Transform, clone)(tf::Transform *self)
{
  return new tf::Transform(*self);
}

TFIMP(void, Transform, delete)(tf::Transform *self)
{
  delete self;
}

TFIMP(void, Transform, setIdentity)(tf::Transform *self)
{
  self->setIdentity();
}

TFIMP(void, Transform, mul_Quaternion)(tf::Transform *self, tf::Quaternion *rot, tf::Quaternion *result)
{
  *result = self->operator*(*rot);
}

TFIMP(void, Transform, mul_Transform)(tf::Transform *self, tf::Transform *other, tf::Transform *result)
{
  *result = self->operator*(*other);
}

TFIMP(void, Transform, inverse)(tf::Transform *self, tf::Transform *result)
{
  *result = self->inverse();
}

TFIMP(void, Transform, getBasis)(tf::Transform *self, THDoubleTensor *basis)
{
  viewMatrix3x3(self->getBasis(), basis);
}

TFIMP(void, Transform, getOrigin)(tf::Transform *self, THDoubleTensor *origin)
{
  viewVector3(self->getOrigin(), origin);
}

TFIMP(void, Transform, setRotation)(tf::Transform *self, tf::Quaternion *rotation)
{
  self->setRotation(*rotation);
}

TFIMP(void, Transform, getRotation)(tf::Transform *self, tf::Quaternion *rotation)
{
  *rotation = self->getRotation();
}
