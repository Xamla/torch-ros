local ffi = require 'ffi'
local torch = require 'torch'
local ros = require 'ros.env'
local utils = require 'ros.utils'
local std = ros.std

local console = {}

function init()
  local names = {
    'initialize',
    'shutdown',
    'set_logger_level',
    'get_loggers',
    'get_logger',
    'print'
  }

  return utils.create_method_table("ros_Console_", names)
end

local f = init()

console.level = {
  Debug = 0,
  Info = 1,
  Warn = 2,
  Error = 3,
  Fatal= 4,
}

function console.initialize()
  f.initialize()
end

function console.shutdown()
  f.shutdown()
end

function console.set_logger_level(name, level)
  f.set_logger_level(name, level)
end

function console.get_loggers()
  local names = std.StringVector()
  local levels = torch.ShortTensor()
  f.get_loggers(names:cdata(), levels:cdata())
  return names, levels
end

function console.get_logger(name)
  return f.get_logger(name)
end

function console.print(logger, level, text, file, function_name, line)
  f.print(logger or ffi.NULL, level, text, file or '??', function_name or '??', line or 0)
end

ros.console = console

local function create_trace(level)
  return function(...)
    local msg = string.format(...)
    local caller = debug.getinfo(2, 'nSl')
    console.print(nil, level, msg, caller.short_src, caller.name, caller.currentline or caller.linedefined)
  end
end

local function create_trace_named(level)
  return function(name, ...)
    local logger = console.get_logger(name)
    local msg = string.format(...)
    local caller = debug.getinfo(2, 'nSl')
    console.print(logger, level, msg, caller.short_src, caller.name, caller.currentline or caller.linedefined)
  end
end

local function create_trace_conditional(level)
  return function(cond, ...)
    if cond then
      local msg = string.format(...)
      local caller = debug.getinfo(2, 'nSl')
      console.print(nil, level, msg, caller.short_src, caller.name, caller.currentline or caller.linedefined)
    end
  end
end

local function create_trace_conditional_named(level)
  return function(cond, name, ...)
    if cond then
      local logger = console.get_logger(name)
      local msg = string.format(...)
      local caller = debug.getinfo(2, 'nSl')
      console.print(logger, level, msg, caller.short_src, caller.name, caller.currentline or caller.linedefined)
    end
  end
end

local function create_logger(postfix, fn_builder)
  local name_level = {
    DEBUG = console.level.Debug,
    INFO = console.level.Info,
    WARN = console.level.Warn,
    ERROR = console.level.Error,
    FATAL = console.level.Fatal
  }
  for k,v in pairs(name_level) do
    ros[k .. postfix] = fn_builder(v)
  end
end

-- declare basic logging helpers
create_logger('', create_trace)
create_logger('_NAMED', create_trace_named)
create_logger('_COND', create_trace_conditional)
create_logger('_COND_NAMED', create_trace_conditional_named)
