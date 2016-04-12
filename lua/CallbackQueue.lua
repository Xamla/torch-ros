local ffi = require 'ffi'
local torch = require 'torch'
local ros = require 'ros.env'
local utils = require 'ros.utils'

local CallbackQueue = torch.class('ros.CallbackQueue', ros)

function init()
  local CallbackQueue_method_names = {
    'new',
    'delete',
    'callOne',
    'callAvailable',
    'isEmpty',
    'clear',
    'enable',
    'disable',
    'isEnabled'
  }

  return utils.create_method_table("ros_CallbackQueue_", CallbackQueue_method_names)
end

local f = init()

function CallbackQueue:__init(enabled)
  self.o = f.new(enabled or true)
end

function CallbackQueue:cdata()
  return self.o
end

function CallbackQueue:callOne(timeout)
  if timeout and not torch.isTypeOf(ros.Duration, timeout) then
    timeout = ros.Duration(timeout)
  end
  return f.callOne(self.o, utils.cdata(timeout))
end

function CallbackQueue:callAvailable(timeout)
  if timeout and not torch.isTypeOf(ros.Duration, timeout) then
    timeout = ros.Duration(timeout)
  end
  f.callAvailable(self.o, utils.cdata(timeout))
end

function CallbackQueue:isEmpty()
  return f.isEmpty(self.o)
end

function CallbackQueue:clear()
  f.clear(self.o)
end

function CallbackQueue:enable()
  f.enable(self.o)
end

function CallbackQueue:disable()
  f.disable(self.o)
end

function CallbackQueue:isEnabled()
  return f.isEnabled(self.o)
end

function CallbackQueue:__tostring()
  return string.format("CallbackQueue {empty: %s, enabled: %s}", self:isEmpty(), self:isEnabled())
end
