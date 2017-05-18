--- ROS main class
-- @classmod ros
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

--- Initialize ROS
-- @tparam[opt=torch_ros] string name Name
-- @param[opt] options Options
function ros.init(name, options, args)
  local args = args or {}
  assert(type(args) == "table", "Arguments should be of type Table. See https://github.com/torch/torch7/blob/master/doc/cmdline.md")

  if not name then
    name = 'torch_ros'
    options = ros.init_options.AnonymousName
  end
  local result = {}
  for i,v in pairs(args) do
      table.insert(result,v)
  end
  local arg
  if #args > 1 then
    arg = ffi.new(string.format("const char*[%d]",#args),args)
  end
  f.init(name, options or 0, #result, arg)
end

---  Will call all the callbacks waiting to be called at the moment.
-- @tparam[opt=false] bool no_default_callbacks If true, the callbacks waiting in the default callback queue will not be called
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

--- Register a callback.
-- @tparam func fn Callback function
-- @tparam int round Callbacks with a higher round integer are called after all callbacks with lower round numbers have been called.
function ros.registerSpinCallback(fn, round)
  ros.DEFAULT_CALLBACK_QUEUE:registerSpinCallback(fn, round)
end

--- Unregister a callback
-- @tparam func fn Callback function
-- @tparam int round Callbacks with a higher round integer are called after all callbacks with lower round numbers have been called.
function ros.unregisterSpinCallback(fn, round)
  ros.DEFAULT_CALLBACK_QUEUE:unregisterSpinCallback(fn, round)
end

return ros
