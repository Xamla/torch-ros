local ffi = require 'ffi'
local torch = require 'torch'
local ros = require 'ros.env'
local utils = require 'ros.utils'

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

  local f = utils.create_method_table("ros___", ros_method_names)

  for n,v in pairs(f) do
    ros[n] = v
  end

  return f
end

local f = init()

ros.init_options = {
  NoSigintHandler = 1,
  AnonymousName = 2,
  NoRosout = 4
}

function ros.init(name, options)
  if not name then
    name = 'torch_ros'
    options = ros.init_options.AnonymousName
  end
  f.init(name, options or 0)
end

function ros.spinOnce(no_default_callbacks)
  f.spinOnce()

  if not no_default_callbacks then
    -- process pending callbacks on default queue
    local queue = ros.DEFAULT_CALLBACK_QUEUE
    if queue ~= nil and ros.ok() then
      queue:callAvailable()
    end
  end
end

-- callbacks with a higher round integer are called after all callbacks with lower round numbers have been called.
function ros.registerSpinCallback(fn, round)
  ros.DEFAULT_CALLBACK_QUEUE:registerSpinCallback(fn, round)
end

function ros.unregisterSpinCallback(fn, round)
  ros.DEFAULT_CALLBACK_QUEUE:unregisterSpinCallback(fn, round)
end

return ros
