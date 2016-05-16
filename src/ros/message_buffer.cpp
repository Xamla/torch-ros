#include "torch-ros.h"
#include "../std/torch-std.h"
#include "message_buffer.h"

#define MessageBuffer_ptr boost::shared_ptr<MessageBuffer>

ROSIMP(MessageBuffer_ptr*, MessageBuffer, new)(int max_backlog) {
  return new MessageBuffer_ptr(new MessageBuffer(max_backlog));
}

ROSIMP(void, MessageBuffer, delete)(MessageBuffer_ptr *self) {
  delete self;
}

ROSIMP(int, MessageBuffer, count)(MessageBuffer_ptr *self) {
  return static_cast<int>((*self)->count());
}

ROSIMP(void, MessageBuffer, clear)(MessageBuffer_ptr *self) {
  (*self)->clear();
}

ROSIMP(bool, MessageBuffer, read)(MessageBuffer_ptr *self, int timeout_milliseconds, THByteStorage *msg_output, StringMap *header_output) {
  boost::shared_ptr<RawMessage> buffer = (*self)->read(timeout_milliseconds);
  if (!buffer)
    return false;

  // copy message to output byte storage
  if (msg_output) {
    THByteStorage_resize(msg_output, buffer->get_length());
    uint8_t* dst = THByteStorage_data(msg_output);
    memcpy(dst, buffer->get_buffer().get(), buffer->get_length());
  }

  if (header_output) {
    *header_output = buffer->get_header();
  }

  return true;
}
