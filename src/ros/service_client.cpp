#include "torch-ros.h"
#include "../std/torch-std.h"
#include "raw_message.h"

ROSIMP(ros::ServiceClient *, ServiceClient, new)(
  const char *service_name,
  bool persistent,
  StringMap *header_values,
  const char *service_md5sum
) {
  return new ros::ServiceClient(service_name, persistent, *header_values, service_md5sum);
}

ROSIMP(ros::ServiceClient *, ServiceClient, clone)(ros::ServiceClient *self) {
  return new ros::ServiceClient(*self);
}

ROSIMP(void, ServiceClient, delete)(ros::ServiceClient *ptr) {
  delete ptr;
}

ROSIMP(bool, ServiceClient, call)(ros::ServiceClient *self, THByteStorage *request_msg, THByteStorage *response_msg, const char *service_md5sum) {
  // fill request message from request_msg byte storage
  RawMessage msg;
  msg.copyFrom(THByteStorage_data(request_msg), THByteStorage_size(request_msg));
  ros::SerializedMessage req(msg.get_buffer(), msg.get_length());
  ros::SerializedMessage resp;

  bool result = self->call(req, resp, service_md5sum);

  // copy response message back to response_msg byte storage
  THByteStorage_resize(response_msg, resp.num_bytes);
  if (resp.num_bytes > 0) {
    uint8_t *response_data = THByteStorage_data(response_msg);
    memcpy(response_data, resp.message_start, resp.num_bytes);
  }

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
