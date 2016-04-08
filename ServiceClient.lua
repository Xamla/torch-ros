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

function ServiceClient:__init(service_name, service_spec, persistent, header_values)
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

  if not torch.isTypeOf(request_msg, ros.Message) then
    -- support filling request message from simple value or table
    if type(request_msg) ~= 'table' then
      request_msg = { request_msg }
    end
    local req = self:createRequest()
    req:fillFromTable(request_msg)
    request_msg = req
  end

  response_msg = ros.Message(self.spec.response_spec, true)

  local v = request_msg:serialize()
  v:shrinkToFit()

  local response_bytes = torch.ByteStorage()
  local result = f.call(self.o, v.storage:cdata(), response_bytes:cdata(), self.spec:md5())
  if result == true then
    response_msg:deserialize(response_bytes)
  else
    response_msg = nil
  end

  return response_msg
end

function ServiceClient:isPersistent()
  return f.isPersistent(self.o)
end

function ServiceClient:getService()
  local s = std.String()
  f.getService(self.o, s:cdata())
  return s
end

function ServiceClient:waitForExistence(timeout)
  if timeout and not torch.isTypeOf(ros.Duration, timeout) then
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
