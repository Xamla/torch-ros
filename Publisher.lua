local ffi = require 'ffi'
local torch = require 'torch'
local ros = require 'ros.env'
local utils = require 'ros.utils'
local tf = ros.tf

local Publisher = torch.class('ros.Publisher', ros)
local Publisher_ptr_ct = ffi.typeof('ros_Publisher *')

function init()
  local Publisher_method_names = {
    'clone',
    'delete',
    'shutdown',
    'getTopic',
    'getNumSubscribers',
    'isLatched',
    'publish'
  }

  f = utils.create_method_table('ros_Publisher_', Publisher_method_names)
end

init()

function Publisher:__init(ptr)
  if not ptr or not ffi.typeof(ptr) == Publisher_ptr_ct then
    error('argument 1: ros::Publisher * expected.')
  end
  self.o = ptr
  ffi.gc(ptr, f.delete)
end

function Publisher:cdata()
  return self.o
end

function Publisher:clone()
  local c = torch.factory('ros.Publisher')()
  rawset(c, 'o', f.clone(self.o))
  return c
end

function Publisher:shutdown()
  f.shutdown(self.o)
end

function Publisher:getTopic()
  return ffi.string(f.getTopic(self.o))
end

function Publisher:getNumSubscribers()
  return f.getNumSubscribers(self.o)
end

function Publisher:isLatched()
  return f.isLatched(self.o)
end

function Publisher:publish(msg)
  -- serialize message to byte storage
  v = msg:serialize()
  v:shrinkToFit()
  f.publish(self.o, v.storage:cdata(), 0, v.length)
end
