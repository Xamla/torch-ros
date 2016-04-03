local ffi = require 'ffi'
local torch = require 'torch'
local ros = require 'ros.env'
local utils = require 'ros.utils'
local std = ros.std

local master = {}
ros.master = master

function init()
  local names = {
    'execute',
    'getHost',
    'getPort',
    'getURI',
    'check',
    'getTopics',
    'getNodes',
    'setRetryTimeout'
  }

  return utils.create_method_table("ros_Master_", names)
end

local f = init()

function master.execute(method, request, wait_for_master)
  if not torch.isTypeOf(request, std.Variable) then
    request = std.Variable(request)
  end
  local response, payload = std.Variable(), std.Variable()
  local result = f.execute(method, request:cdata(), response:cdata(), payload:cdata(), wait_for_master or false)
  return result, response:get(), payload:get()
end

function master.getHost()
  return ffi.string(f.getHost())
end

function master.getPort()
  return f.getPort()
end

function master.getURI()
  return ffi.string(f.getURI())
end

function master.check()
  return f.check()
end

function master.getTopics(output)
  local v = output or std.VariableTable()
  f.getTopics(v:cdata())
  return v:totable()
end

function master.getNodes(output)
  local v = output or std.StringVector()
  f.getNodes(v:cdata())
  return v
end

function master.setRetryTimeout(_1, _2)
  local d = _1
  if not torch.isTypeOf(d, ros.Duration) then
    d = ros.Duration(_1, _2)
  end
  f.setRetryTimeout(d:get_sec(), d:get_nsec())
end
