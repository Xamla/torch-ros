#include "torch-ros.h"
#include <boost/algorithm/string.hpp>
#include <ros/callback_queue.h>
#include "../std/torch-std.h"
#include "message_buffer.h"
#include "../utils.h"

ROSIMP(ros::NodeHandle *, NodeHandle, new)(const char *ns, ros::NodeHandle *parent, StringMap *remappings) {
  if (parent != NULL) {
    if (remappings != NULL)
      return new ros::NodeHandle(*parent, ns, *remappings);
    else
      return new ros::NodeHandle(*parent, ns);
  } else {
    if (remappings != NULL)
      return new ros::NodeHandle(ns, *remappings);
    else
      return new ros::NodeHandle(ns);
  }
}

ROSIMP(void, NodeHandle, delete)(ros::NodeHandle *self) {
  delete self;
}

ROSIMP(void, NodeHandle, shutdown)(ros::NodeHandle *self) {
  self->shutdown();
}

ROSIMP(bool, NodeHandle, ok)(ros::NodeHandle *self) {
  return self->ok();
}

ROSIMP(const char *, NodeHandle, getNamespace)(ros::NodeHandle *self) {
  return self->getNamespace().c_str();
}

ROSIMP(const char *, NodeHandle, getUnresolvedNamespace)(ros::NodeHandle *self) {
  return self->getUnresolvedNamespace().c_str();
}

ROSIMP(void, NodeHandle, resolveName)(ros::NodeHandle *self, const char *name, bool remap, std::string *result) {
  *result = self->resolveName(name, remap);
}

ROSIMP(ros::Subscriber *, NodeHandle, subscribe)(
  ros::NodeHandle *self,
  boost::shared_ptr<MessageBuffer> *message_buffer,
  const char *topic,
  unsigned int queue_size,
  const char *md5sum,
  const char *datatype,
  StringVector *transports,
  StringMap *transport_options
) {
  ros::SubscribeOptions so(topic, queue_size, md5sum, datatype);
  if (transports != NULL) {
    for (StringVector::const_iterator i = transports->begin(); i != transports->end(); ++i) {
      const std::string& transport = *i;
      if (boost::iequals(transport, "udp") || boost::iequals(transport, "unreliable"))
        so.transport_hints.udp();
      else if (boost::iequals(transport, "tcp") || boost::iequals(transport, "reliable"))
        so.transport_hints.tcp();
    }
  }
  if (transport_options != NULL) {
    for (StringMap::const_iterator i = transport_options->begin(); i != transport_options->end(); ++i) {
      const std::string& key = i->first;
      const std::string& value = i->second;
      if (boost::iequals(key, "tcp_nodelay") || boost::iequals(key, "tcpnodelay"))
        so.transport_hints.tcpNoDelay(boost::iequals(value, "true"));
      else if (boost::iequals(key, "maxDatagramSize"))
        so.transport_hints.maxDatagramSize(boost::lexical_cast<int>(value));
    }
  }
  so.helper = *message_buffer;
  return new ros::Subscriber(self->subscribe(so));
}

typedef void (*_ServiceStatusCallback)(const char *subscriber_name, const char *subscriber_topic);

static void ServiceStatusCallbackAdapter(_ServiceStatusCallback cb, const ros::SingleSubscriberPublisher &ssp) {
  cb(ssp.getSubscriberName().c_str(), ssp.getTopic().c_str());
}

ROSIMP(ros::Publisher *, NodeHandle, advertise)(
  ros::NodeHandle *self,
  const char *topic,
  unsigned int queue_size,
  const char *md5sum,
  const char *datatype,
  const char *message_definition,
  bool has_header,
  bool latch,
  _ServiceStatusCallback connect_cb,
  _ServiceStatusCallback disconnect_cb,
  ros::CallbackQueue *callback_queue
) {
  ros::AdvertiseOptions ao(topic, queue_size, md5sum, datatype, message_definition);
  ao.callback_queue = callback_queue;
  ao.has_header = has_header;
  ao.latch = latch;
  if (connect_cb) {
    ao.connect_cb = boost::bind(ServiceStatusCallbackAdapter, connect_cb, _1);
  }
  if (disconnect_cb) {
    ao.disconnect_cb = boost::bind(ServiceStatusCallbackAdapter, disconnect_cb, _1);
  }
  return new ros::Publisher(self->advertise(ao));
}

ROSIMP(ros::ServiceClient *, NodeHandle, serviceClient)(
  ros::NodeHandle *self,
  const char *service,
  const char *md5sum,
  bool persistent,
  StringMap *header
) {
  static const StringMap empty_header;
  ros::ServiceClientOptions ops(service, md5sum, persistent, header != NULL ? *header : empty_header);
  return new ros::ServiceClient(self->serviceClient(ops));
}

typedef bool (*ServiceRequestCallback)(THByteStorage *, THByteStorage *, std::map<std::string, std::string> *);

class ServiceRequestCallbackHandler
 : public ros::ServiceCallbackHelper
{
public:
  ServiceRequestCallbackHandler(ServiceRequestCallback callback)
    : callback(callback) {
  }

  virtual bool call(ros::ServiceCallbackHelperCallParams &params) {
    THByteStorage *response = THByteStorage_new();

    // copy request data
    THByteStorage *request = THByteStorage_newWithSize(params.request.num_bytes + sizeof(uint32_t));
    uint8_t *request_data = THByteStorage_data(request);
    ros::serialization::OStream stream(request_data, THByteStorage_size(request));
    stream.next((uint32_t)params.request.num_bytes);
    memcpy(stream.advance(params.request.num_bytes), params.request.message_start, params.request.num_bytes);

    bool result = this->callback(request, response, params.connection_header.get());

    // copy response
    long response_length = THByteStorage_size(response);
    if (response_length > 0) {
      uint8_t *response_data = THByteStorage_data(response);
      boost::shared_array<uint8_t> dst(new uint8_t[response_length]);
      memcpy(dst.get(), response_data, response_length);
      params.response = ros::SerializedMessage(dst, response_length);
    } else {
      params.response = ros::SerializedMessage();
    }

    THByteStorage_free(response);
    THByteStorage_free(request);

    return result;
  }

private:
  ServiceRequestCallback callback;
};


ROSIMP(ros::ServiceServer*, NodeHandle, advertiseService)(
  ros::NodeHandle *self,
  const char *service,
  const char *md5sum,
  const char *datatype,
  const char *req_datatype,
  const char *res_datatype,
  ServiceRequestCallback callback,
  ros::CallbackQueue *callback_queue
) {
  ros::AdvertiseServiceOptions ops;

  ops.service = service;
  ops.md5sum = md5sum;
  ops.datatype = datatype;            // package/ServiceName
  ops.req_datatype = req_datatype;    // package/ServiceNameRequest
  ops.res_datatype = res_datatype;    // package/ServiceNameResponse

  ops.helper.reset(new ServiceRequestCallbackHandler(callback));
  ops.callback_queue = callback_queue;

  const ros::ServiceServer &srv = self->advertiseService(ops);
  return new ros::ServiceServer(srv);
}

ROSIMP(bool, NodeHandle, hasParam)(ros::NodeHandle *self, const char *key) {
  return self->hasParam(key);
}

ROSIMP(bool, NodeHandle, deleteParam)(ros::NodeHandle *self, const char *key) {
  return self->deleteParam(key);
}

ROSIMP(bool, NodeHandle, getParamString)(ros::NodeHandle *self, const char *key, std::string *result) {
  return self->getParam(key, *result);
}

ROSIMP(bool, NodeHandle, getParamDouble)(ros::NodeHandle *self, const char *key, double *result) {
  return self->getParam(key, *result);
}

ROSIMP(bool, NodeHandle, getParamFloat)(ros::NodeHandle *self, const char *key, float *result) {
  return self->getParam(key, *result);
}

ROSIMP(bool, NodeHandle, getParamInt)(ros::NodeHandle *self, const char *key, int *result) {
  return self->getParam(key, *result);
}

ROSIMP(bool, NodeHandle, getParamBool)(ros::NodeHandle *self, const char *key, bool *result) {
  return self->getParam(key, *result);
}

ROSIMP(void, NodeHandle, setParamString)(ros::NodeHandle *self, const char *key, const char *value) {
  self->setParam(key, std::string(value));
}

ROSIMP(void, NodeHandle, setParamDouble)(ros::NodeHandle *self, const char *key, double value) {
  self->setParam(key, value);
}

ROSIMP(void, NodeHandle, setParamFloat)(ros::NodeHandle *self, const char *key, float value) {
  self->setParam(key, value);
}

ROSIMP(void, NodeHandle, setParamInt)(ros::NodeHandle *self, const char *key, int value) {
  self->setParam(key, value);
}

ROSIMP(void, NodeHandle, setParamBool)(ros::NodeHandle *self, const char *key, bool value) {
  self->setParam(key, value);
}

ROSIMP(bool, NodeHandle, getParamStringVector)(ros::NodeHandle *self, const char *key, std::vector<std::string> *result) {
  return self->getParam(key, *result);
}

ROSIMP(bool, NodeHandle, getParamBoolVector)(ros::NodeHandle *self, const char *key, THByteTensor *result) {
  std::vector<bool> v;
  bool ok = self->getParam(key, v);

  THByteTensor_resize1d(result, v.size());
  THByteTensor* output_ = THByteTensor_newContiguous(result);
  std::copy(v.begin(), v.end(), THByteTensor_data(output_));
  THByteTensor_freeCopyTo(output_, result);

  return ok;
}

ROSIMP(bool, NodeHandle, getParamIntVector)(ros::NodeHandle *self, const char *key, THIntTensor *result) {
  std::vector<int> v;
  bool ok = self->getParam(key, v);
  vector2Tensor(v, result);
  return ok;
}

ROSIMP(bool, NodeHandle, getParamDoubleVector)(ros::NodeHandle *self, const char *key, THDoubleTensor *result) {
  std::vector<double> v;
  bool ok = self->getParam(key, v);
  vector2Tensor(v, result);
  return ok;
}

ROSIMP(bool, NodeHandle, getParamFloatVector)(ros::NodeHandle *self, const char *key, THFloatTensor *result) {
  std::vector<float> v;
  bool ok = self->getParam(key, v);
  vector2Tensor(v, result);
  return ok;
}

ROSIMP(void, NodeHandle, setParamStringVector)(ros::NodeHandle *self, const char *key, std::vector<std::string> *value) {
  self->setParam(key, *value);
}

ROSIMP(void, NodeHandle, setParamBoolVector)(ros::NodeHandle *self, const char *key, THByteTensor *value) {
  std::vector<bool> v;

  long n = THByteTensor_nElement(value);
  v.resize(n);
  value = THByteTensor_newContiguous(value);
  unsigned char *begin = THByteTensor_data(value);
  std::copy(begin, begin + n, v.begin());
  THByteTensor_free(value);

  self->setParam(key, v);
}

ROSIMP(void, NodeHandle, setParamIntVector)(ros::NodeHandle *self, const char *key, THIntTensor *value) {
  std::vector<int> v;
  Tensor2vector(value, v);
  self->setParam(key, v);
}

ROSIMP(void, NodeHandle, setParamDoubleVector)(ros::NodeHandle *self, const char *key, THDoubleTensor *value) {
  std::vector<double> v;
  Tensor2vector(value, v);
  self->setParam(key, v);
}

ROSIMP(void, NodeHandle, setParamFloatVector)(ros::NodeHandle *self, const char *key, THFloatTensor *value) {
  std::vector<float> v;
  Tensor2vector(value, v);
  self->setParam(key, v);
}
