#include "torch-tf.h"

TFIMP(tf::StampedTransform *, StampedTransform, new)(
  tf::Transform *transform,
  ros::Time* timestamp,
  const char *frame_id,
  const char *child_frame_id
) {
  return new tf::StampedTransform(
    transform ? *transform : tf::Transform::getIdentity(), 
    timestamp ? *timestamp : ros::Time(), 
    frame_id ? frame_id : "", 
    child_frame_id ? child_frame_id : ""
  );
}

TFIMP(tf::StampedTransform *, StampedTransform, clone)(tf::StampedTransform *self) {
  return new tf::StampedTransform(*self);
}

TFIMP(void, StampedTransform, delete)(tf::StampedTransform *self) {
  delete self;
}

TFIMP(tf::Transform *, StampedTransform, getBasePointer)(tf::StampedTransform *self) {
  return static_cast<tf::Transform*>(self);
}

TFIMP(void, StampedTransform, get_stamp)(tf::StampedTransform *self, ros::Time *result) {
  *result = self->stamp_;
}

TFIMP(void, StampedTransform, set_stamp)(tf::StampedTransform *self, ros::Time *stamp) {
  self->stamp_ = *stamp;
}

TFIMP(const char *, StampedTransform, get_frame_id)(tf::StampedTransform *self) {
  return self->frame_id_.c_str();
}

TFIMP(void, StampedTransform, set_frame_id)(tf::StampedTransform *self, const char *id) {
  self->frame_id_ = id;
}

TFIMP(const char *, StampedTransform, get_child_frame_id)(tf::StampedTransform *self) {
  return self->child_frame_id_.c_str();
}

TFIMP(void, StampedTransform, set_child_frame_id)(tf::StampedTransform *self, const char *id) {
  self->child_frame_id_ = id;
}

TFIMP(void, StampedTransform, setData)(tf::StampedTransform *self, tf::Transform *input) {
  self->setData(*input);
}

TFIMP(bool, StampedTransform, eq)(tf::StampedTransform *self, tf::StampedTransform *other) {
  return *self == *other;
}

TFIMP(void, StampedTransform,toStampedTransformMsg)(tf::StampedTransform *self, THByteStorage *output)
{
  geometry_msgs::TransformStamped msg;
  tf::transformStampedTFToMsg(*self, msg);

  uint32_t length = ros::serialization::serializationLength(msg);
  THByteStorage_resize(output, length + sizeof(uint32_t));
  ros::serialization::OStream stream(THByteStorage_data(output), length + sizeof(uint32_t));
  stream.next((uint32_t)length);
  ros::serialization::serialize(stream, msg);
}

TFIMP(void, StampedTransform,toStampedPoseMsg)(tf::StampedTransform *self, THByteStorage *output)
{
  const tf::Pose tf_(self->getRotation(), self->getOrigin());

  geometry_msgs::PoseStamped msg;
  tf::Stamped<tf::Pose> pose (tf_, self->stamp_,self->frame_id_);
  tf::poseStampedTFToMsg(pose, msg);

  uint32_t length = ros::serialization::serializationLength(msg);
  THByteStorage_resize(output, length + sizeof(uint32_t));
  ros::serialization::OStream stream(THByteStorage_data(output), length + sizeof(uint32_t));
  stream.next((uint32_t)length);
  ros::serialization::serialize(stream, msg);
}