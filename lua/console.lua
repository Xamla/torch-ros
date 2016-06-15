--- Wrapper for the ROS logging system
-- @classmod console

local ffi = require 'ffi'
local torch = require 'torch'
local ros = require 'ros.env'
local utils = require 'ros.utils'
local std = ros.std

local console = {}
ros.console = console

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
-- @param name string Name of the logger
-- @param level New log level
-- @param no_default_prefix bool, optional, default=false
function console.set_logger_level(name, level, no_default_prefix)
  f.set_logger_level(name, level, no_default_prefix or false)
end

--- Returns the list of all loggers
-- @return std.StringVector Names of all loggers
-- @return torch.ShortTensor Log level of the loggers
function console.get_loggers()
  local names = std.StringVector()
  local levels = torch.ShortTensor()
  f.get_loggers(names:cdata(), levels:cdata())
  return names, levels
end

--- Get a logger
-- @param  name string Name of the logger
-- @param  no_default_prefix bool, optional, default=false
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
create_logger('_COND_NAMED', create_trace_conditional_named)

console.initialize()
