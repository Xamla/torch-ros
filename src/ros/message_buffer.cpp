#include "torch-ros.h"
#include "message_buffer.h"

#define MessageBuffer_ptr boost::shared_ptr<MessageBuffer>

ROSIMP(MessageBuffer_ptr*, MessageBuffer, new)(int max_backlog) {
  return new MessageBuffer_ptr(new MessageBuffer());
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

ROSIMP(bool, MessageBuffer, read)(MessageBuffer_ptr *self, int timeout_milliseconds, THByteStorage *output) {
  boost::shared_ptr<RawMessage> buffer = (*self)->read(timeout_milliseconds);
  if (!buffer)
    return false;

  // copy message to output byte storage
  THByteStorage_resize(output, buffer->get_length());
  uint8_t* dst = THByteStorage_data(output);
  memcpy(dst, buffer->get_buffer().get(), buffer->get_length());

  return true;
}
