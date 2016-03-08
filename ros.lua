local ffi = require 'ffi'
local torch = require 'torch'
local ros = require 'ros.env'
local utils = require 'ros.utils'

local f

function init()
  local ros_method_names = {
    "init",
    "shutdown",
    "spinOnce",
    "requestShutdown",
    "isInitialized",
    "isStarted",
    "isShuttingDown",
    "ok",
    "waitForShutdown"
  }
  
  f = utils.create_method_table("ros___", ros_method_names)
  
  for n,v in pairs(f) do 
    ros[n] = v
  end
end

init()

ros.init_options = {
  NoSigintHandler = 1,
  AnonymousName = 2,
  NoRosout = 4
}

function ros.init(name, options)
  if not name then
    name = 'torch-ros'
    options = ros.init_options.AnonymousName
  end
  f.init(name, options or 0)
end

return ros
