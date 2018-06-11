local ffi = require 'ffi'
local torch = require 'torch'
local ros = require 'ros.env'
local utils = require 'ros.utils'
local std = ros.std

local ServiceClient = torch.class('ros.ServiceClient', ros)
local ServiceClient_ptr_ct = ffi.typeof('ros_ServiceClient *')

function init()
  local ServiceClient_method_names = {
    "new",
    "clone",
    "delete",
    "call",
    "isPersistent",
    "getService",
    "waitForExistence",
    "exists",
    "shutdown",
    "isValid"
  }

  return utils.create_method_table("ros_ServiceClient_", ServiceClient_method_names)
end

local f = init()

function ServiceClient:__init(service_name, service_spec, persistent, header_values, serialization_handlers)
  self.serialization_handlers = serialization_handlers
  if ffi.istype(ServiceClient_ptr_ct, service_name) then
    self.o = service_name
    self.spec = service_spec
    ffi.gc(self.o, f.delete)
  else
    if type(service_spec) == 'string' then
      service_spec = ros.SrvSpec(service_spec)
    end
    if not torch.isTypeOf(service_spec, ros.SrvSpec) then
      error("ServiceClient:ctor(): invalid 'service_spec' argument.")
    end
    self.spec = service_spec
    self.o = f.new(service_name, persistent or false, utils.cdata(header_values), self.spec:md5())
  end
end

function ServiceClient:clone()
  local c = torch.factory('ros.ServiceClient')()
  rawset(c, 'o', f.clone(self.o))
  rawset(c, 'spec', self.spec)
  return c
end

function ServiceClient:createRequest()
  return ros.Message(self.spec.request_spec)
end

function ServiceClient:call(request_msg)
  if not self:isValid() then
    error('ros.ServiceClient instance is not valid.')
  end

  local sw = ros.StorageWriter(nil, 0, self.serialization_handlers)
  if not torch.isTypeOf(request_msg, ros.Message) then
    -- support filling request message from simple value or table
    if type(request_msg) ~= 'table' then
      request_msg = { request_msg }
    end
    local req = self:createRequest()
    req:fillFromTable(request_msg)
    request_msg = req
  end

  request_msg:serialize(sw)
  sw:shrinkToFit()

  local response_serialized_msg = ros.SerializedMessage()
  local result = f.call(self.o, sw.storage:cdata(), response_serialized_msg:cdata(), self.spec:md5())
  local response_msg
  if result == true then
    local view = response_serialized_msg:view()
    local storage = view:storage() or torch.ByteStorage()   -- storage may be nil if response is empty messages
    local sr = ros.StorageReader(storage, view:storageOffset()-1, nil, nil, self.serialization_handlers)
    response_msg = ros.Message(self.spec.response_spec, true)
    response_msg:deserialize(sr, true)    -- true singals not that no total length was prepended to message
  end

  return response_msg
end

function ServiceClient:isPersistent()
  return f.isPersistent(self.o)
end

function ServiceClient:getService()
  local s = std.String()
  f.getService(self.o, s:cdata())
  return s:get()
end

function ServiceClient:waitForExistence(timeout)
  if timeout and not torch.isTypeOf(timeout, ros.Duration) then
    timeout = ros.Duration(timeout)
  end
  return f.waitForExistence(self.o, utils.cdata(timeout))
end

function ServiceClient:exists()
  return f.exists(self.o)
end

function ServiceClient:shutdown()
  f.shutdown(self.o)
end

function ServiceClient:isValid()
  return f.isValid(self.o)
end
