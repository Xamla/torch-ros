--- ROS main class
-- @classmod ros
local ffi = require 'ffi'
local torch = require 'torch'
local ros = require 'ros.env'
local utils = require 'ros.utils'
local std = ros.std

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
-- @param[opt] int options Options
-- @param[opt] tab remappings Table with with key->value (local_name->external_name) remappings or list with command line args of which entries containing ':=' will be passed as remappings to ros::init().
function ros.init(name, options, remappings)
  remappings = remappings or {}
  assert(type(remappings) == "table", "Argument 'remappings' must be of type table. See https://github.com/torch/torch7/blob/master/doc/cmdline.md")

  local remap = std.StringMap()
  for k,v in pairs(remappings) do
    if type(k) == 'string' and type(v) == 'string' then
      remap[k] = v
    elseif type(k) == 'number' then
      local local_name, external_name = string.match(v, '(.*):=(.*)')
      if local_name and external_name then
        ros.DEBUG("remap: %s => %s", local_name, external_name)
        remap[local_name] = external_name
      end
    end
  end

  if not name then
    name = 'torch_ros'
    options = ros.init_options.AnonymousName
  end
  f.init(remap:cdata(), name, options or 0)
end

---  Will call all the callbacks waiting to be called at the moment.
-- @tparam[opt=true] ros.Duration. Time to wait for callback.
-- @tparam[opt=true] bool no_default_callbacks If true, the callbacks waiting in the default callback queue will not be called
function ros.spinOnce(timeout, no_default_callbacks)
  if torch.type(timeout) ==  'boolean' then
    no_default_callbacks = timeout
    timeout = nil
  elseif torch.type(timeout) ==  'number' then
    timeout = ros.Duration(timeout)
  end
  f.spinOnce()

  if not no_default_callbacks then
    -- process pending callbacks on default queue
    local queue = ros.DEFAULT_CALLBACK_QUEUE
    if queue ~= nil and ros.ok() then
      queue:callAvailable(timeout)
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
