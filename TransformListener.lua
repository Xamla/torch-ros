local ffi = require 'ffi'
local torch = require 'torch'
local ros = require 'ros.env'
local utils = require 'ros.utils'
local std = ros.std
local tf = ros.tf

local TransformListener = torch.class('tf.TransformListener', tf)

local f

function init()
  local TransformListener_method_names = {
    "new",
    "delete",
    "clear",
    "getFrameStrings",
    "lookupTransform",
    "waitForTransform",
    "canTransform",
    "lookupTransformFull",
    "waitForTransformFull",
    "canTransformFull",
    "resolve",
    "getLatestCommonTime",
    "chainAsVector",
    "getParent",
    "frameExists",
    "getCacheLength",
    "getTFPrefix"
  }
  
  f = utils.create_method_table("tf_TransformListener_", TransformListener_method_names)
end

init()

function TransformListener:__init()
  self.o = f.new()
end

function TransformListener:clear()
  f.clear(self.o)
end

function TransformListener:getFrameStrings()
  local result = std.StringVector()
  f.getFrameStrings(self.o, result:cdata())
  return result
end

function TransformListener:lookupTransform(target_frame, source_frame, time, result)
  result = result or tf.StampedTransform()
  f.lookupTransform(self.o, target_frame, source_frame, time:cdata(), result:cdata())
  return result
end

function TransformListener:waitForTransform(target_frame, source_frame, time, timeout, may_throw)
  timeout = timeout or ros.Duration(10)
  if may_throw then
    local error_msg = std.String()
    if not f.waitForTransform(self.o, target_frame, source_frame, time:cdata(), timeout:cdata(), error_msg:cdata()) then
      error(error_msg:get())
    end
    return true
  else
    return f.waitForTransform(self.o, target_frame, source_frame, time:cdata(), timeout:cdata(), ffi.NULL)
  end
end

function TransformListener:canTransform(target_frame, source_frame, time)
  return f.canTransform(self.o, target_frame, source_frame, time:cdata())
end

function TransformListener:lookupTransformFull(target_frame, target_time, source_frame, source_time, fixed_frame, result)
  result = result or tf.StampedTransform()
  f.lookupTransformFull(self.o, target_frame, target_time:cdata(), source_frame, source_time:cdata(), fixed_frame, result:cdata())
  return result
end

function TransformListener:waitForTransformFull(target_frame, target_time, source_frame, source_time, fixed_frame, timeout, may_throw)
  timeout = timeout or ros.Duration(10)
  if may_throw then
    local error_msg = std.String()
    if not f.waitForTransformFull(self.o, target_frame, target_time:cdata(), source_frame, source_time:cdata(), fixed_frame, timeout:cdata(), error_msg:cdata()) then
      error(error_msg:get())
    end
    return true
  else
    return f.waitForTransformFull(self.o, target_frame, target_time:cdata(), source_frame, source_time:cdata(), fixed_frame, timeout:cdata(), ffi.NULL)
  end
end

function TransformListener:canTransformFull(target_frame, target_time, source_frame, source_time, fixed_frame)
  return f.canTransformFull(self.o, target_frame, target_time:cdata(), source_frame, source_time:cdata(), fixed_frame)
end

function TransformListener:resolve(frame_name)
  local name = std.String()
  f.resolve(self.o, frame_name, name:cdata())
  return name:get()
end

function TransformListener:getLatestCommonTime(source_frame, target_frame)
  local result = ros.Time()
  local error_msg = std.String()
  if t.getLatestCommonTime(self.o, source_frame, target_frame, result:cdata(), error_msg:cdata()) == 0 then
    error(error_msg:get())
  end
end

function TransformListener:chainAsVector(target_frame, target_time, source_frame, source_time, fixed_frame, result)
  result = result or std.StringVector
  t.chainAsVector(self.o, target_frame, target_time:cdata(), source_frame, source_time:cdata(), fixed_frame, result:cdata())
  return result
end

function TransformListener:getParent(frame_id, time)
  time = time or ros.Time(0)
  local parent = std.String()
  t.getParent(self.o, frame_id, time:cdata(), parent:cdata())
  return parent:get()
end

function TransformListener:frameExists(frame_id)
  return t.frameExists(self.o, frame_id)
end

function TransformListener:getCacheLength()
  local duration = ros.Duration()
  t.getCacheLength(self.o, duration:cdata())
  return duration
end

function TransformListener:getTFPrefix()
  local prefix = std.String()
  t.getTFPrefix(self.o, prefix:cdata())
  return prefix
end
