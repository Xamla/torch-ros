--- Wrapper for the ros::AsyncSpinner class.
-- This spinner spins asynchronously when you call start(), and stops
-- when either you call stop(), ros::shutdown() is called, or its
-- destructor is called AsyncSpinner is reference counted internally,
-- so if you copy one it will continue spinning until all copies have
-- destructed (or stop() has been called on one of them)
-- @classmod AsyncSpinner

local ffi = require 'ffi'
local torch = require 'torch'
local ros = require 'ros.env'
local utils = require 'ros.utils'

local AsyncSpinner = torch.class('ros.AsyncSpinner', ros)

function init()
  local Duration_method_names = {
    "new",
    "delete",
    "canStart",
    "start",
    "stop"
  }

  return utils.create_method_table("ros_AsyncSpinner_", Duration_method_names)
end

local f = init()

--- Constructor
-- @tparam[opt=0] int thread_count The number of threads to use. A value of 0 means to use the number of processor cores.
function AsyncSpinner:__init(thread_count)
  self.o = f.new(thread_count or 0)
end

--- Check if the spinner can be started.
-- A spinner can't be started if another spinner is already running.
-- @treturn bool true if the spinner could be started, false otherwise
function AsyncSpinner:canStart()
  return f.canStart(self.o)
end

--- Start this spinner spinning asynchronously.
function AsyncSpinner:start()
  f.start(self.o)
end

--- Stop this spinner from running.
function AsyncSpinner:stop()
  f.stop(self.o)
end
