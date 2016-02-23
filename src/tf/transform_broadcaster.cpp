#include "torch-tf.h"
#include <tf/transform_broadcaster.h>

TFIMP(tf::TransformBroadcaster *, TransformBroadcaster, new)()
{
  return new tf::TransformBroadcaster();
}

TFIMP(void, TransformBroadcaster, delete)(tf::TransformBroadcaster *self)
{
  delete self;
}

TFIMP(void, TransformBroadcaster, sendTransform)(tf::TransformBroadcaster *self, tf::StampedTransform *transform)
{
  self->sendTransform(*transform);
}
