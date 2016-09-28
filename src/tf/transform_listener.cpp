#include "torch-tf.h"
#include <tf/transform_listener.h>

TFIMP(tf::TransformListener *, TransformListener, new)() {
  return new tf::TransformListener();
}

TFIMP(void, TransformListener, delete)(tf::TransformListener *self) {
  delete self;
}

TFIMP(void, TransformListener, clear)(tf::TransformListener *self) {
  self->clear();
}

TFIMP(void, TransformListener, getFrameStrings)(tf::TransformListener *self, std::vector<std::string> *result) {
  try
    {
      self->getFrameStrings(*result);
    }
  catch (std::runtime_error& e)
    {
      ROS_ERROR("Exception: [%s]", e.what());
    }
}

TFIMP(void, TransformListener, lookupTransform)(
  tf::TransformListener *self,
  const char *target_frame,
  const char *source_frame, ros::Time *time,
  tf::StampedTransform *result
) {
  try
    {
      self->lookupTransform(target_frame, source_frame, *time, *result);
    }
  catch (std::runtime_error& e)
    {
      ROS_ERROR("Exception: [%s]", e.what());
    }
}

TFIMP(bool, TransformListener, waitForTransform)(
  tf::TransformListener *self,
  const char *target_frame,
  const char *source_frame,
  ros::Time *time,
  ros::Duration *timeout,
  std::string *error_msg
) {
  return self->waitForTransform(target_frame, source_frame, *time, *timeout, ros::Duration(0.01), error_msg);
}

TFIMP(bool, TransformListener, canTransform)(
  tf::TransformListener *self,
  const char *target_frame,
  const char *source_frame,
  ros::Time *time
) {
  return self->canTransform(target_frame, source_frame, *time, NULL);
}

TFIMP(void, TransformListener, lookupTransformFull)(tf::TransformListener *self,
  const char *target_frame, ros::Time *target_time,
  const char *source_frame, ros::Time *source_time,
  const char *fixed_frame, tf::StampedTransform *result
  ) {
  self->lookupTransform(target_frame, *target_time, source_frame, *source_time, fixed_frame, *result);
}

TFIMP(bool, TransformListener, waitForTransformFull)(tf::TransformListener *self,
  const char *target_frame, ros::Time *target_time,
  const char *source_frame, ros::Time *source_time,
  const char *fixed_frame, ros::Duration *timeout, std::string *error_msg
) {
  return self->waitForTransform(target_frame, *target_time, source_frame, *source_time, fixed_frame, *timeout, ros::Duration(0.01), error_msg);
}

TFIMP(bool, TransformListener, canTransformFull)(tf::TransformListener *self,
  const char *target_frame, ros::Time *target_time,
  const char *source_frame, ros::Time *source_time,
  const char *fixed_frame
) {
  return self->canTransform(target_frame, *target_time, source_frame, *source_time, fixed_frame, NULL);
}

TFIMP(void, TransformListener, resolve)(tf::TransformListener *self, const char *frame_name, std::string *result) {
  *result = self->resolve(frame_name);
}

TFIMP(int, TransformListener, getLatestCommonTime)(
  tf::TransformListener *self,
  const char *source_frame,
  const char *target_frame,
  ros::Time *time,
  std::string *error_string
) {
  return self->getLatestCommonTime(source_frame, target_frame, *time, error_string);
}

TFIMP(void, TransformListener, chainAsVector)(tf::TransformListener *self,
  const char *target_frame, ros::Time *target_time,
  const char *source_frame, ros::Time *source_time,
  const char *fixed_frame, std::vector<std::string> *result
) {
  self->chainAsVector(target_frame, *target_time, source_frame, *source_time, fixed_frame, *result);
}

TFIMP(bool, TransformListener, getParent)(
  tf::TransformListener *self,
  const char* frame_id,
  ros::Time *time,
  std::string *result
) {
  return self->getParent(frame_id, *time, *result);
}

TFIMP(bool, TransformListener, frameExists)(tf::TransformListener *self, const char *frame_id) {
  try
    {
      return self->frameExists(frame_id);
    }
  catch (std::runtime_error& e)
    {
      ROS_ERROR("Exception: [%s]", e.what());
      return false;
    }
}

TFIMP(void, TransformListener, getCacheLength)(tf::TransformListener *self, ros::Duration *result) {
  *result = self->getCacheLength();
}

TFIMP(void, TransformListener, getTFPrefix)(tf::TransformListener *self, std::string *result) {
  *result = self->getTFPrefix();
}
