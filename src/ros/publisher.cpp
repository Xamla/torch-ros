#include "torch-ros.h"
#include "raw_message.h"

ROSIMP(ros::Publisher*, Publisher, clone)(ros::Publisher *self) {
  return new ros::Publisher(*self);
}

ROSIMP(void, Publisher, delete)(ros::Publisher *self) {
  delete self;
}

ROSIMP(void, Publisher, shutdown)(ros::Publisher *self) {
  self->shutdown();
}

ROSIMP(void, Publisher, getTopic)(ros::Publisher *self, std::string *output) {
  *output = self->getTopic();
}

ROSIMP(int, Publisher, getNumSubscribers)(ros::Publisher *self) {
  return static_cast<int>(self->getNumSubscribers());
}

ROSIMP(bool, Publisher, isLatched)(ros::Publisher *self) {
  return self->isLatched();
}

ROSIMP(void, Publisher, publish)(ros::Publisher *self, THByteStorage *serialized_msg, ptrdiff_t offset, size_t length) {
  RawMessage msg;
  long storage_size = THByteStorage_size(serialized_msg);
  if (offset + length > static_cast<size_t>(storage_size) || storage_size < 0)
    throw std::range_error("Specified array segment lies outside buffer.");
  msg.copyFrom(THByteStorage_data(serialized_msg) + offset, length);
  self->publish(msg);
}
