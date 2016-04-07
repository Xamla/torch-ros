#include "torch-ros.h"
#include "../std/torch-std.h"
#include "message_buffer.h"
#include "../utils.h"

ROSIMP(ros::NodeHandle *, NodeHandle, new)(const char *ns) {
  return new ros::NodeHandle(ns);
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
  const char *datatype
) {
  ros::SubscribeOptions so(topic, queue_size, md5sum, datatype);
  so.helper = *message_buffer;
  return new ros::Subscriber(self->subscribe(so));
}

ROSIMP(ros::Publisher *, NodeHandle, advertise)(
  ros::NodeHandle *self,
  const char *topic,
  unsigned int queue_size,
  const char *md5sum,
  const char *datatype,
  const char *message_definition
) {
  ros::AdvertiseOptions ao(topic, queue_size, md5sum, datatype, message_definition);
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

ROSIMP(ros::ServiceServer*, NodeHandle, advertiseService)(
  ros::NodeHandle *self,
  const char *service,
  const char *md5sum,
  const char *datatype,
  const char *req_datatype,
  const char *res_datatype
) {
  ros::AdvertiseServiceOptions ops;

  ops.service = service;
  ops.md5sum = md5sum;
  ops.datatype = datatype;            // package/ServiceName
  ops.req_datatype = req_datatype;    // package/ServiceNameRequest
  ops.res_datatype = res_datatype;    // package/ServiceNameResponse

  //ops.helper = ;

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
