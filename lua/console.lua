--- Wrapper for the ROS logging system
-- @module console

local ffi = require 'ffi'
local torch = require 'torch'
local ros = require 'ros.env'
local utils = require 'ros.utils'
local std = ros.std

local console = {}
ros.console = console
console.trace_once_stack = {}
console.trace_throttle_stack = {}
function init()
  local names = {
    'initialize',
    'shutdown',
    'set_logger_level',
    'get_loggers',
    'check_loglevel',
    'get_logger',
    'print'
  }

  return utils.create_method_table("ros_Console_", names)
end

local f = init()

console.Level = {
  Debug = 0,
  Info  = 1,
  Warn  = 2,
  Error = 3,
  Fatal = 4,
}

--- Don't call this function directly. Performs any required initialization/configuration.
-- This function is called automatically during construction of the object.
function console.initialize()
  console.NAME_PREFIX = ffi.string(f.initialize())
end

--- Shut down the console
function console.shutdown()
  f.shutdown()
end

--- Set the log level for a given logger
-- @tparam string name Name of the logger
-- @param level New log level
-- @tparam[opt=false] bool no_default_prefix
function console.set_logger_level(name, level, no_default_prefix)
  f.set_logger_level(name, level, no_default_prefix or false)
end

--- Returns the list of all loggers
-- @treturn std.StringVector Names of all loggers
-- @treturn torch.ShortTensor Log level of the loggers
function console.get_loggers()
  local names = std.StringVector()
  local levels = torch.ShortTensor()
  f.get_loggers(names:cdata(), levels:cdata())
  return names, levels
end

--- Get a logger
-- @tparam  string name Name of the logger
-- @tparam[opt=false]  bool no_default_prefix
-- @return Pointer to the logger
function console.get_logger(name, no_default_prefix)
  return f.get_logger(name, no_default_prefix or false)
end

function console.check_loglevel(name, level, no_default_prefix)
  return f.check_loglevel(name, level, no_default_prefix or false)
end

-- aliases
console.setLoggerLevel = console.set_logger_level
console.getLoggers = console.get_loggers
console.checkLogLevel = console.check_loglevel

--- TODO: docu
function console.print(logger, level, text, file, function_name, line)
  f.print(logger or ffi.NULL, level, text, file or '??', function_name or '??', line or 0)
end

local function create_trace(level)
  return function(...)
    if console.check_loglevel(nil, level) then
      local msg = string.format(...)
      local caller = debug.getinfo(2, 'nSl')
      console.print(nil, level, msg, caller.short_src, caller.name, caller.currentline or caller.linedefined)
    end
  end
end

local function create_trace_named(level)
  return function(name, ...)
    if console.check_loglevel(name, level) then
      local logger = console.get_logger(name)
      local msg = string.format(...)
      local caller = debug.getinfo(2, 'nSl')
      console.print(logger, level, msg, caller.short_src, caller.name, caller.currentline or caller.linedefined)
    end
  end
end

local function create_trace_conditional(level)
  return function(cond, ...)
    if cond and console.check_loglevel(nil, level) then
      local msg = string.format(...)
      local caller = debug.getinfo(2, 'nSl')
      console.print(nil, level, msg, caller.short_src, caller.name, caller.currentline or caller.linedefined)
    end
  end
end

local function create_trace_conditional_named(level)
  return function(cond, name, ...)
    if cond and console.check_loglevel(name, level) then
      local logger = console.get_logger(name)
      local msg = string.format(...)
      local caller = debug.getinfo(2, 'nSl')
      console.print(logger, level, msg, caller.short_src, caller.name, caller.currentline or caller.linedefined)
    end
  end
end

local function create_trace_once_conditional_named(level)
  return function(key, cond, name, ...)
    if not console.trace_stack[key] and cond and console.check_loglevel(name, level) then
      console.trace_stack[key] = true
      local logger = console.get_logger(name)
      local msg = string.format(...)
      local caller = debug.getinfo(2, 'nSl')
      console.print(logger, level, msg, caller.short_src, caller.name, caller.currentline or caller.linedefined)
    end
  end
end

local function create_trace_once_named(level)
  return function(key, name, ...)
    if not console.trace_once_stack[key] and console.check_loglevel(nil, level) then
      console.trace_once_stack[key] = true
      local logger = console.get_logger(name)
      local msg = string.format(...)
      local caller = debug.getinfo(2, 'nSl')
      console.print(logger, level, msg, caller.short_src, caller.name, caller.currentline or caller.linedefined)
    end
  end
end

local function create_trace_once(level)
  return function(key, ...)
    if not console.trace_once_stack[key] and console.check_loglevel(nil, level) then
      console.trace_once_stack[key] = true
      local msg = string.format(...)
      local caller = debug.getinfo(2, 'nSl')
      console.print(nil, level, msg, caller.short_src, caller.name, caller.currentline or caller.linedefined)
    end
  end
end

local function create_trace_throttle(level)
  return function(key, period, ...)
    if console.check_loglevel(nil, level) then
      local msg = string.format(...)
      local caller = debug.getinfo(2, 'nSl')
      if not console.trace_throttle_stack[key] then
        console.trace_throttle_stack[key] = ros.Time.now()
        console.print(nil, level, msg, caller.short_src, caller.name, caller.currentline or caller.linedefined)
      else
        if (ros.Time.now() - console.trace_throttle_stack[key]):toSec() > period then
          console.print(nil, level, msg, caller.short_src, caller.name, caller.currentline or caller.linedefined)
          console.trace_throttle_stack[key] = ros.Time.now()
        end
      end
    end
  end
end

local function create_trace_throttle_named(level)
  return function(key, period, name, ...)
    if console.check_loglevel(nil, level) then
      local msg = string.format(...)
      local caller = debug.getinfo(2, 'nSl')
      local logger = console.get_logger(name)
      if not console.trace_throttle_stack[key] then
        console.trace_throttle_stack[key] = ros.Time.now()
        console.print(nil, level, msg, caller.short_src, caller.name, caller.currentline or caller.linedefined)
      else
        if (ros.Time.now() - console.trace_throttle_stack[key]):toSec() > period then
          console.print(logger, level, msg, caller.short_src, caller.name, caller.currentline or caller.linedefined)
          console.trace_throttle_stack[key] = ros.Time.now()
        end
      end
    end
  end
end

local function create_logger(postfix, fn_builder)
  local name_level = {
    DEBUG = console.Level.Debug,
    INFO = console.Level.Info,
    WARN = console.Level.Warn,
    ERROR = console.Level.Error,
    FATAL = console.Level.Fatal
  }
  for k,v in pairs(name_level) do
    local fn = fn_builder(v)
    ros[k .. postfix] = fn
    ros['ROS_' .. k .. postfix] = fn
  end
end

-- declare basic logging helpers
create_logger('', create_trace)
create_logger('_NAMED', create_trace_named)
create_logger('_COND', create_trace_conditional)
create_logger('_THROTTLE', create_trace_throttle)
create_logger('_ONCE', create_trace_once)
create_logger('_COND_NAMED', create_trace_conditional_named)
create_logger('_THROTTLE_NAMED', create_trace_throttle_named)
create_logger('_ONCE_NAMED', create_trace_once_named)

console.initialize()
