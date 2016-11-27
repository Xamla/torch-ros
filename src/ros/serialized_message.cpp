#include "torch-ros.h"
#include "../std/torch-std.h"
#include "message_buffer.h"


ROSIMP(ros::SerializedMessage*, SerializedMessage, new)() {
  return new ros::SerializedMessage();
}

ROSIMP(void, SerializedMessage, delete)(ros::SerializedMessage *self) {
  delete self;
}

ROSIMP(void, SerializedMessage, view)(ros::SerializedMessage *self, THByteTensor *view) {
  if (self->buf.get() == NULL || self->num_bytes == 0) {
    THByteTensor_resize1d(view, 0);
  } else {
    // creae special storage that views into memory of serialized message object
    THByteStorage *storage = THByteStorage_newWithData(self->buf.get(), self->num_bytes);
    storage->flag = TH_STORAGE_REFCOUNTED;
    ptrdiff_t offset = self->message_start - self->buf.get();
    THByteTensor_setStorage1d(view, storage, offset, self->num_bytes, 1);
  }
}

ROSIMP(int, SerializedMessage, size)(ros::SerializedMessage *self) {
  return static_cast<int>(self->num_bytes);
}

ROSIMP(uint8_t*, SerializedMessage, data)(ros::SerializedMessage *self) {
  return self->buf.get();
}

ROSIMP(void, SerializedMessage, resize)(ros::SerializedMessage *self, size_t new_size) {
  const size_t old_size = self->num_bytes;
  if (old_size == new_size)
    return;   // nothing to do

  if (new_size > old_size) {
    // reallocate & copy exstinig data
    boost::shared_array<uint8_t> old_buf(self->buf);
    boost::shared_array<uint8_t> new_buf(new uint8_t[new_size]);
    memcpy(new_buf.get(), old_buf.get(), std::min(new_size, old_size));
    self->buf = new_buf;
    self->message_start = new_buf.get() + (self->message_start - old_buf.get());
  } else {
    // do not reallocate when shrinking
  }

  self->num_bytes = new_size;
}
