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
    'waitCallAvailable',
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
  self.spin_callbacks = { {}, {}, {}, {}, {} }
end

function CallbackQueue:cdata()
  return self.o
end

function CallbackQueue:registerSpinCallback(fn, round)
  self.spin_callbacks[round or 1][fn] = true -- table used as set
end

function CallbackQueue:unregisterSpinCallback(fn, round)
  self.spin_callbacks[round or 1][fn] = nil
end

function CallbackQueue:callSpinCallbacks()
  for i,cbs in ipairs(self.spin_callbacks) do
    local isolation_copy = utils.getTableKeys(cbs)   -- changes of spin_callbacks become effecitve after iteration
    for _,f in ipairs(isolation_copy) do
      if not ros.ok() then return end
      f()
    end
  end
end

function CallbackQueue:callOne(timeout)
  if timeout and not torch.isTypeOf(ros.Duration, timeout) then
    timeout = ros.Duration(timeout)
  end
  return f.callOne(self.o, utils.cdata(timeout))
end

function CallbackQueue:callAvailable(timeout, no_spin_callbacks)
  if timeout and not torch.isTypeOf(ros.Duration, timeout) then
    timeout = ros.Duration(timeout)
  end
  if not self:isEmpty() then
    f.callAvailable(self.o, utils.cdata(timeout))
  end
  if not no_spin_callbacks then
    self:callSpinCallbacks()
  end
end

function CallbackQueue:waitCallAvailable(timeout)
  if timeout and not torch.isTypeOf(ros.Duration, timeout) then
    timeout = ros.Duration(timeout)
  end
  return f.waitCallAvailable(self.o, utils.cdata(timeout))
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

ros.DEFAULT_CALLBACK_QUEUE = ros.CallbackQueue()
