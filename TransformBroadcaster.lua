local ffi = require 'ffi'
local torch = require 'torch'
local ros = require 'ros.env'
local utils = require 'ros.utils'
local tf = ros.tf

local TransformBroadcaster = torch.class('tf.TransformBroadcaster', tf)

function init()
  local TransformBroadcaster_method_names = {
    "new",
    "delete",
    "sendTransform"
  }
  
  return utils.create_method_table("tf_TransformBroadcaster_", TransformBroadcaster_method_names)
end

local f = init()

function TransformBroadcaster:__init()
  self.o = f.new()
end

function TransformBroadcaster:sendTransform(stampedTransform)
  f.sendTransform(self.o, stampedTransform:cdata())
end
