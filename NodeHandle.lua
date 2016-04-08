local ffi = require 'ffi'
local torch = require 'torch'
local ros = require 'ros.env'
local utils = require 'ros.utils'
local std = ros.std

local NodeHandle = torch.class('ros.NodeHandle', ros)

function init()
  local NodeHandle_method_names = {
    'new',
    'delete',
    'shutdown',
    'ok',
    'getNamespace',
    'getUnresolvedNamespace',
    'resolveName',
    'subscribe',
    'advertise',
    'serviceClient',
    'advertiseService',
    'hasParam',
    'deleteParam',
    'getParamString',
    'getParamDouble',
    'getParamFloat',
    'getParamInt',
    'getParamBool',
    'setParamString',
    'setParamDouble',
    'setParamFloat',
    'setParamInt',
    'setParamBool',
    'getParamStringVector',
    'getParamBoolVector',
    'getParamIntVector',
    'getParamDoubleVector',
    'getParamFloatVector',
    'setParamStringVector',
    'setParamBoolVector',
    'setParamIntVector',
    'setParamDoubleVector',
    'setParamFloatVector'
  }

  return utils.create_method_table('ros_NodeHandle_', NodeHandle_method_names)
end

local f = init()

function NodeHandle:__init(ns)
  self.o = f.new(ns or '')
end

function NodeHandle:cdata()
  return self.o
end

function NodeHandle:shutdown()
  f.shutdown(self.o)
end

function NodeHandle:ok()
  return f.ok(self.o)
end

function NodeHandle:getNamespace()
  return ffi.string(f.getNamespace(self.o))
end

function NodeHandle:getUnresolvedNamespace()
  return ffi.string(f.getUnresolvedNamespace(self.o))
end

function NodeHandle:resolveName(name, remap)
  local result = std.String()
  f.resolveName(self.o, name, remap or true, result:cdata())
  return result
end

function NodeHandle:subscribe(topic, msg_spec, queue_size)
  if type(msg_spec) == 'string' then
    msg_spec = ros.MsgSpec(msg_spec)
  end
  buffer = buffer or ros.MessageBuffer()
  local s = f.subscribe(self.o, buffer:cdata(), topic, queue_size or 1000, msg_spec:md5(), msg_spec.type)
  return ros.Subscriber(s, buffer, msg_spec)
end

function NodeHandle:advertise(topic, msg_spec, queue_size)
  if type(msg_spec) == 'string' then
    msg_spec = ros.MsgSpec(msg_spec)
  end
  local p = f.advertise(self.o, topic, queue_size or 1000, msg_spec:md5(), msg_spec.type, msg_spec.definition)
  return ros.Publisher(p)
end

function NodeHandle:serviceClient(service_name, service_spec, persistent, header_values)
  if type(service_spec) == 'string' then
    service_spec = ros.SrvSpec(service_spec)
  end
  if not torch.isTypeOf(service_spec, ros.SrvSpec) then
    error("NodeHandle:serviceClient(): invalid 'service_spec' argument.")
  end
  local client = f.serviceClient(self.o, service_name, service_spec:md5(), persistent or false, utils.cdata(header_values))
  return ros.ServiceClient(client, service_spec)
end

function NodeHandle:advertiseService(service_name, service_spec, callback_queue, service_handler_func)
  if not torch.isTypeOf(callback_queue, ros.CallbackQueue) then
    error('Explicitly specified callback queue required (Lua is single-threaded).')
  end

  -- create message serialization/deserialization wrapper function
  local function handler(request_storage, response_storage, header_values)

    -- create torch.ByteStorage() obj from THByteStorage* and addref
    request_storage = torch.pushudata(request_storage, 'torch.ByteStorage')
    request_storage:retain()

    response_storage = torch.pushudata(response_storage, 'torch.ByteStorage')
    response_storage:retain()

    -- create class around header values string map...
    local header = torch.factory('std.StringMap')()
    rawset(header, 'o', header_values)

    -- deserialize request
    local request_msg = ros.Message(service_spec.request_spec, true)
    request_msg:deserialize(ros.StorageReader(request_storage))
    local response_msg = ros.Message(service_spec.response_spec)

    -- call actual service handler function
    local status = service_handler_func(request_msg, response_msg, header)

    -- serialize response
    local sw = ros.StorageWriter(response_storage)
    local v = response_msg:serializeServiceResponse(sw, status)
    sw:shrinkToFit()

    return status
  end

  local cb = ffi.cast("ServiceRequestCallback", handler)
  local srv_ptr = f.advertiseService(
    self.o,
    service_name,
    service_spec:md5(),
    service_spec.type,
    service_spec.request_spec.type,
    service_spec.response_spec.type,
    callback_queue:cdata(),
    cb
  )
  return ros.ServiceServer(srv_ptr, cb, service_handler_func)
end

function NodeHandle:hasParam(key)
  return f.hasParam(self.o, key)
end

function NodeHandle:deleteParam(key)
  return f.deleteParam(self.o, key)
end

function NodeHandle:getParamString(key)
  local result = std.String()
  local ok = f.getParamString(self.o, key, result:cdata())
  return result:get(), ok
end

local double_ct = ffi.typeof('double[1]')
local float_ct = ffi.typeof('float[1]')
local int_ct = ffi.typeof('int[1]')
local bool_ct = ffi.typeof('bool[1]')

function NodeHandle:getParamDouble(key)
  local result = double_ct(0)
  local ok = f.setParamDouble(self.o, key, result)
  return result[0], ok
end

function NodeHandle:getParamFloat(key)
  local result = float_ct(0)
  local ok = f.getParamFloat(self.o, key, result)
  return result[0], ok
end

function NodeHandle:getParamInt(key)
  local result = int_ct(0)
  local ok = f.getParamInt(self.o, key, result)
  return result[0], ok
end

function NodeHandle:getParamBool(key)
  local result = bool_ct(0)
  local ok = f.getParamBool(self.o, key, result)
  return result[0], ok
end

function NodeHandle:setParamString(key, value)
  f.setParamString(self.o, key, value)
end

function NodeHandle:setParamDouble(key, value)
  f.setParamDouble(self.o, key, value)
end

function NodeHandle:setParamFloat(key, value)
  f.setParamFloat(self.o, key, value)
end

function NodeHandle:setParamInt(key, value)
  f.setParamInt(self.o, key, value)
end

function NodeHandle:setParamBool(key, value)
  f.setParamBool(self.o, key, value)
end

function NodeHandle:getParamStringVector(key)
  local result = std.StringVector()
  local ok = f.getParamStringVector(self.o, key, result:cdata())
  return result, ok
end

function NodeHandle:getParamBoolVector(key)
  local result = torch.ByteTensor()
  local ok = f.getParamBoolVector(self.o, key, result:cdata())
  return result, ok
end

function NodeHandle:getParamIntVector(key)
  local result = torch.IntTensor()
  local ok = f.getParamIntVector(self.o, key, result:cdata())
  return result, ok
end

function NodeHandle:getParamDoubleVector(key)
  local result = torch.DoubleTensor()
  local ok = f.getParamDoubleVector(self.o, key, result:cdata())
  return result, ok
end

function NodeHandle:getParamFloatVector(key)
  local result = torch.FloatTensor()
  local ok = f.getParamDoubleVector(self.o, key, result:cdata())
  return result, ok
end

function NodeHandle:setParamStringVector(key, value)
  f.setParamStringVector(self.o, key, value:cdata())
end

function NodeHandle:setParamBoolVector(key, value)
  f.setParamBoolVector(self.o, key, value:cdata())
end

function NodeHandle:setParamIntVector(key, value)
  f.setParamIntVector(self.o, key, value:cdata())
end

function NodeHandle:setParamDoubleVector(key, value)
  f.setParamDoubleVector(self.o, key, value:cdata())
end

function NodeHandle:setParamFloatVector(key, value)
  f.setParamFloatVector(self.o, key, value:cdata())
end
