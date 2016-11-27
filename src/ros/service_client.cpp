#include "torch-ros.h"
#include "../std/torch-std.h"
#include "raw_message.h"

ROSIMP(ros::ServiceClient *, ServiceClient, new)(
  const char *service_name,
  bool persistent,
  StringMap *header_values,
  const char *service_md5sum
) {
  static const StringMap empty_header;
  return new ros::ServiceClient(service_name, persistent, header_values ? *header_values : empty_header, service_md5sum);
}

ROSIMP(ros::ServiceClient *, ServiceClient, clone)(ros::ServiceClient *self) {
  return new ros::ServiceClient(*self);
}

ROSIMP(void, ServiceClient, delete)(ros::ServiceClient *ptr) {
  delete ptr;
}

ROSIMP(bool, ServiceClient, call)(ros::ServiceClient *self, THByteStorage *request_msg, ros::SerializedMessage *response_msg, const char *service_md5sum) {
  // fill request message from request_msg byte storage
  RawMessage msg;
  msg.copyFrom(THByteStorage_data(request_msg), THByteStorage_size(request_msg));

  ros::SerializedMessage req = ros::serialization::serializeMessage(msg);
  bool result = self->call(req, *response_msg, service_md5sum);

  return result;
}

ROSIMP(bool, ServiceClient, isPersistent)(ros::ServiceClient *self) {
  return self->isPersistent();
}

ROSIMP(void, ServiceClient, getService)(ros::ServiceClient *self, std::string *output) {
  *output = self->getService();
}

ROSIMP(bool, ServiceClient, waitForExistence)(ros::ServiceClient *self, ros::Duration *timeout) {
  return self->waitForExistence(timeout != NULL ? *timeout : ros::Duration(-1));
}

ROSIMP(bool, ServiceClient, exists)(ros::ServiceClient *self) {
  return self->exists();
}

ROSIMP(void, ServiceClient, shutdown)(ros::ServiceClient *self) {
  self->shutdown();
}

ROSIMP(bool, ServiceClient, isValid)(ros::ServiceClient *self) {
  return self->isValid();
}
