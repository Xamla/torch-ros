#ifndef raw_message_h
#define raw_message_h

class RawMessage {
public:
  boost::shared_array<uint8_t> buf;
  size_t num_bytes;

  RawMessage()
    : num_bytes(0) {
  }

  void assign(uint8_t *source, size_t length) {
    buf = boost::shared_array<uint8_t>(new uint8_t[length]);
    memcpy(buf.get(), source, length);
    num_bytes = length;
  }
};

namespace ros {
namespace serialization {

template<>
inline SerializedMessage serializeMessage(const RawMessage &message)
{
  return SerializedMessage(message.buf, message.num_bytes);
}

}   // namespace serialization
}   // namespace ros

#endif    // raw_message_h
