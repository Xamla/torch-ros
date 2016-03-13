local ffi = require 'ffi'
local torch = require 'torch'
local ros = require 'ros.env'
local utils = require 'ros.utils'
local std = ros.std

local MessageBuffer = torch.class('ros.MessageBuffer', ros)

function init()
  local MessageBuffer_method_names = {
    "new",
    "delete",
    "count",
    "clear",
    "read"
  }
  
  return utils.create_method_table("ros_MessageBuffer_", MessageBuffer_method_names)
end

local f = init()

function MessageBuffer:__init(max_backlog)
  self.o = f.new(max_backlog or -1)
end

function MessageBuffer:cdata()
  return self.o
end

function MessageBuffer:getCount()
  return f.count(self.o)
end

function MessageBuffer:clear()
  f.clear(self.o)
end

function MessageBuffer:read(timeout_milliseconds, result)
  result = result or torch.ByteStorage()
  if not f.read(self.o, timeout_milliseconds or 100, result:cdata()) then
    return nil
  end
  return result
end
