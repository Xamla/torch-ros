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

function Subscriber:__init(ptr, buffer, msg_spec, callback_queue, serialization_handlers)
  if not ptr or not ffi.typeof(ptr) == Subscriber_ptr_ct then
    error('argument 1: ros.Subscriber * expected.')
  end
  self.o = ptr
  self.buffer = buffer
  self.msg_spec = msg_spec
  ffi.gc(ptr, f.delete)
  self.callback_queue = callback_queue or ros.DEFAULT_CALLBACK_QUEUE
  self.callbacks = {}
  self.serialization_handlers = serialization_handlers
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
  if self.spin_callback_function ~= nil then
    ros.unregisterSpinCallback(self.spin_callback_function)
    self.spin_callback_function = nil
  end

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
  local msg_bytes, msg_header = self.buffer:read(timeout_milliseconds)
  local msg
  if msg_bytes then
    local sr = ros.StorageReader(msg_bytes, 0, nil, nil, self.serialization_handlers)

    local handler = sr:getHandler(self.msg_spec.type)
    if handler ~= nil then
      local totalLength = sr:readUInt32()
      msg = handler:read(sr)
    else
      msg = result or ros.Message(self.msg_spec, true)
      msg:deserialize(sr)
    end
  end
  return msg, msg_header
end

function Subscriber:triggerCallbacks()
  local cbs
  local count = self:getMessageCount()
  while count > 0 do
    count=count-1
    local msg, header = self:read(0)
    if msg ~= nil then
      cbs = cbs or utils.getTableKeys(self.callbacks)   -- lazy isolation copy of callbacks
      for _,f in ipairs(cbs) do
        f(msg, header, self)
      end
    end
  end
end

function Subscriber:registerCallback(message_cb)
  self.callbacks[message_cb] = true -- table used as set
  if self.spin_callback_function == nil then
    self.spin_callback_function = function() self:triggerCallbacks() end
    self.spin_callback_id = self.callback_queue:registerSpinCallback(self.spin_callback_function)
  end
end

function Subscriber:unregisterCallback(message_cb)
  self.callbacks[message_cb] = nil
  if self.spin_callback_function ~= nil and next(self.callbacks) == nil then
    self.callback_queue:unregisterSpinCallback(self.spin_callback_function)
    self.spin_callback_function = nil
  end
end
