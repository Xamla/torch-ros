local ffi = require 'ffi'
local torch = require 'torch'
local ros = require 'ros.env'
local utils = require 'ros.utils'
local std = ros.std

local NodeHandle = torch.class('ros.NodeHandle', ros)

local f

function init()
  local NodeHandle_method_names = {
    "new",
    "delete",
    "subscribe"
  }
  
  f = utils.create_method_table("ros_NodeHandle_", NodeHandle_method_names)
end

function NodeHandle:__init()
  self.o = f.new()
end

function NodeHandle:cdata()
  return self.o
end

function NodeHandle:suscribe(topic, msg_spec, queue_size)
  if type(msg_spec) == 'string' then
    msg_spec = ros.MsgSpec(msg_spec)
  end
  local s = f.subscribe(self.o, queue_size or 1000, msg_spec:md5(), msg_spec.type)
  return ros.Subscriber(s)
end
