local ffi = require 'ffi'
local torch = require 'torch'
local ros = require 'ros.env'
local utils = require 'ros.utils'
local tf = ros.tf

local Subscriber = torch.class('ros.Subscriber', ros)
local Subscriber_ptr_ct = ffi.typeof('ros_Subscriber *')

function init()
  local Subscriber_method_names = {
    "clone",
    "delete",
    "shutdown",
    "getTopic",
    "getNumPublishers"
  }
  
  return utils.create_method_table("ros_Subscriber_", Subscriber_method_names)
end

local f = init()

function Subscriber:__init(ptr, buffer, msg_spec)
  if not ptr or not ffi.typeof(ptr) == Subscriber_ptr_ct then
    error('argument 1: ros.Subscriber * expected.')
  end
  self.o = ptr
  self.buffer = buffer
  self.msg_spec = msg_spec
  ffi.gc(ptr, f.delete)
end

function Subscriber:cdata()
  return self.o
end

function Subscriber:clone()
  local c = torch.factory('ros.Subscriber')() 
  rawset(c, 'o', f.clone(self.o))
  return c
end

function Subscriber:shutdown()
  f.shutdown(self.o)
end

function Subscriber:getTopic()
  return ffi.string(f.getTopic(self.o))
end

function Subscriber:getNumPublishers()
  return f.getNumPublishers(self.o)
end

function Subscriber:hasMessage()
  return self:getMessageCount() > 0
end

function Subscriber:getMessageCount()
  return self.buffer:getCount()
end

function Subscriber:read(timeout_milliseconds, result)
  local msg_bytes = self.buffer:read(timeout_milliseconds)
  local msg
  if msg_bytes then
    msg = result or ros.Message(self.msg_spec, true)
    msg:deserialize(msg_bytes)
  end
  return msg
end
