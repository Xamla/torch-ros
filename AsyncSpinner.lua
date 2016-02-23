local ffi = require 'ffi'
local torch = require 'torch'
local ros = require 'ros.env'
local utils = require 'ros.utils'

local AsyncSpinner = torch.class('ros.AsyncSpinner', ros)

local f

function init()
  local Duration_method_names = {
    "new",
    "delete",
    "canStart",
    "start",
    "stop"
  }
  
  f = utils.create_method_table("ros_AsyncSpinner_", Duration_method_names)
end

init()

function AsyncSpinner:__init(thread_count)
  self.o = f.new(thread_count or 0)
end

function AsyncSpinner:canStart()
  return f.canStart(self.o)
end

function AsyncSpinner:start()
  f.start(self.o)
end

function AsyncSpinner:stop()
  f.stop(self.o)
end
