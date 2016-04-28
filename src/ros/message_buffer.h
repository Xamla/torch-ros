#ifndef message_buffer_h
#define message_buffer_h

#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/thread/locks.hpp>
#include <deque>
#include "raw_message.h"

class MessageBuffer
  : public ros::SubscriptionCallbackHelper {
public:
  MessageBuffer(int max_backlog = -1)
    : max_backlog(max_backlog) {
  }

  virtual ros::VoidConstPtr deserialize(const ros::SubscriptionCallbackHelperDeserializeParams &params) {

    // create buffer and copy message bytes
    boost::shared_ptr<RawMessage> buffer(new RawMessage(params.length + sizeof(uint32_t)));
    ros::serialization::OStream stream(buffer->get_OStream());
    stream.next((uint32_t)params.length);
    memcpy(buffer->get() + sizeof(uint32_t), params.buffer, params.length);

    // lock queue mutex and add new message buffer to queue
    {
      boost::unique_lock<boost::mutex> lock(queue_lock);
      if (max_backlog >= 0 && message_queue.size() >= static_cast<size_t>(max_backlog))
        message_queue.pop_front();    // remove oldest message
      message_queue.push_back(buffer);
    }

    // notify potentially waiting thread
    message_available.notify_one();
    return ros::VoidConstPtr();
  }

  virtual void call(ros::SubscriptionCallbackHelperCallParams &params) {
  }

  virtual const std::type_info& getTypeInfo() {
    return typeid(void);
  }

  virtual bool isConst() {
    return false;
  }

  virtual bool hasHeader() {
    return false;
  }

  boost::shared_ptr<RawMessage> read(int timeout_milliseconds) {
    boost::unique_lock<boost::mutex> lock(queue_lock);

    boost::chrono::system_clock::time_point timeout = boost::chrono::system_clock::now()
      + boost::chrono::milliseconds(timeout_milliseconds);

    do {
      // check if messages are available
      if (!message_queue.empty()) {
        boost::shared_ptr<RawMessage> msg = message_queue.front();
        message_queue.pop_front();
        return msg;
      }

      // wait with timeout for frame to be captured
    } while (timeout_milliseconds != 0
      && (timeout_milliseconds < 0 || message_available.wait_until(lock, timeout) != boost::cv_status::timeout)
    );

    return boost::shared_ptr<RawMessage>();
  }

  size_t count() {
    boost::unique_lock<boost::mutex> lock(queue_lock);
    return message_queue.size();
  }

  void clear() {
    boost::unique_lock<boost::mutex> lock(queue_lock);
    message_queue.clear();
  }

private:
  int max_backlog;          // unlimited if maxBacklog < 0
  boost::mutex queue_lock;
  boost::condition_variable message_available;
  std::deque<boost::shared_ptr<RawMessage> > message_queue;
};

#endif   // message_buffer_h
