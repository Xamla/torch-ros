local ffi = require 'ffi'
local torch = require 'torch'
local ros = require 'ros.env'
local utils = require 'ros.utils'
local std = ros.std

local this_node = {}
ros.this_node = this_node

function init()
  local names = {
    'getName',
    'getNamespace',
    'getAdvertisedTopics',
    'getSubscribedTopics'
  }

  return utils.create_method_table("ros_ThisNode_", names)
end

local f = init()

function this_node.getName()
  return ffi.string(f.getName())
end

function this_node.getNamespace()
  return ffi.string(f.getNamespace())
end

function this_node.getAdvertisedTopics(result)
  result = result or std.StringVector()
  f.getAdvertisedTopics(result:cdata())
  return result
end

function this_node.getSubscribedTopics(result)
  result = result or std.StringVector()
  f.getSubscribedTopics(result:cdata())
  return result
end
