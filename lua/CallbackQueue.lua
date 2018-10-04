--- Callback queue to handle the callbacks within ROS
-- @classmod CallbackQueue

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

--- Constructor
-- @tparam[opt=true] bool enabled indicating if the queue is enabled or disabled after construction
function CallbackQueue:__init(enabled)
  self.o = f.new(enabled or true)
  self.spin_callbacks = { {}, {}, {}, {}, {} }
end

--- Get the underlying c++ instance
-- @return pointer to the c++ instance
function CallbackQueue:cdata()
  return self.o
end

--- Register a function for callback
-- @tparam func fn The function to execute as callback function
-- @tparam[opt=1] int round Prioriy/order of the callback. If unsure which value to use, omit the parameter
function CallbackQueue:registerSpinCallback(fn, round)
  local list = self.spin_callbacks[round or 1]
  if utils.indexOf(list, fn) < 0 then
    list[#list+1] = fn
  end
end

--- Remove a function from the callback list
-- @tparam func fn The function to be removed
-- @tparam[opt=1] int round ; Prioriy/order of the when added. Has to be the same value as used for registerSpinCallback()
function CallbackQueue:unregisterSpinCallback(fn, round)
  local list = self.spin_callbacks[round or 1]
  local i = utils.indexOf(list, fn)
  if i >= 0 then
    table.remove(list, i)
  end
end

--- Trigger all callbacks one time
function CallbackQueue:callSpinCallbacks()
  for i,cbs in ipairs(self.spin_callbacks) do
    local isolation_copy = utils.cloneList(cbs)   -- changes of spin_callbacks become effecitve after iteration
    for _,f in ipairs(isolation_copy) do
      if not ros.ok() then return end
      f()
    end
  end
end
jit.off(CallbackQueue.callSpinCallbacks)

--- Pop a single callback off the front of the queue and invoke it. If the callback was not ready to be called, pushes it back onto the queue.
-- @tparam ?number|ros:Duration timeout Timeout for the callback, either in seconds (fractional numbers like 1.5 possible) or as an instance of ros:Duration
function CallbackQueue:callOne(timeout)
  if timeout and not torch.isTypeOf(timeout, ros.Duration) then
    timeout = ros.Duration(timeout)
  end

  local result = f.callOne(self.o, utils.cdata(timeout))

  return result
end
jit.off(CallbackQueue.callOne)

--- Invoke all callbacks currently in the queue.
-- If a callback was not ready to be called, pushes it back onto the queue.
-- @tparam ?number|ros:Duration timeout Timeout for the callback, either in seconds (fractional numbers like 1.5 possible) or as an instance of ros:Duration
-- @tparam bool no_spin_callbacks
function CallbackQueue:callAvailable(timeout, no_spin_callbacks)
  if timeout and not torch.isTypeOf(timeout, ros.Duration) then
    timeout = ros.Duration(timeout)
  end
  f.callAvailable(self.o, utils.cdata(timeout))

  if not no_spin_callbacks then
    self:callSpinCallbacks()
  end
end
jit.off(CallbackQueue.callAvailable)

--- Waits until the next callback is enqueued or the timeout is expired
-- @tparam ?number|ros:Duration timeout Timeout for the callback, either in seconds (fractional numbers like 1.5 possible) or as an instance of ros:Duration
-- @treturn bool true if a callback is enqueued, false if the queue is disabled or the timeout expired
function CallbackQueue:waitCallAvailable(timeout)
  if timeout and not torch.isTypeOf(timeout, ros.Duration) then
    timeout = ros.Duration(timeout)
  end
  return f.waitCallAvailable(self.o, utils.cdata(timeout))
end
jit.off(CallbackQueue.waitCallAvailable)

--- Returns whether or not the queue is empty
-- @treturn bool true if queue is empty, false otherwise
function CallbackQueue:isEmpty()
  return f.isEmpty(self.o)
end

--- Removes all callbacks from the queue. Does not wait for calls currently in progress to finish.
function CallbackQueue:clear()
  f.clear(self.o)
end

--- Enable the queue
function CallbackQueue:enable()
  f.enable(self.o)
end

--- Disable the queue, meaning any calls to addCallback() will have no effect.
function CallbackQueue:disable()
  f.disable(self.o)
end

--- Returns whether or not this queue is enabled.
-- @treturn bool true if the queue is empty
function CallbackQueue:isEnabled()
  return f.isEnabled(self.o)
end

function CallbackQueue:__tostring()
  return string.format("CallbackQueue {empty: %s, enabled: %s}", self:isEmpty(), self:isEnabled())
end

ros.DEFAULT_CALLBACK_QUEUE = ros.CallbackQueue()
