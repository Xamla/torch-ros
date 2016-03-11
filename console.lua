local ffi = require 'ffi'
local torch = require 'torch'
local ros = require 'ros.env'
local utils = require 'ros.utils'
local std = ros.std

local console = {}

local f

function init()
  local names = {
    'initialize',
    'shutdown',
    'set_logger_level',
    'get_loggers',
    'get_logger',
    'print'
  }

  f = utils.create_method_table("ros_Console_", names)
end

init()

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
