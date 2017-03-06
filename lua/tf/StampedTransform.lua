local ffi = require 'ffi'
local torch = require 'torch'
local ros = require 'ros.env'
local utils = require 'ros.utils'
local tf = ros.tf

local StampedTransform, parent = torch.class('tf.StampedTransform', 'tf.Transform', tf)

function init()
  local StampedTransform_method_names = {
    "new",
    "clone",
    "delete",
    "getBasePointer",
    "get_stamp",
    "set_stamp",
    "get_frame_id",
    "set_frame_id",
    "get_child_frame_id",
    "set_child_frame_id",
    "setData",
    "eq",
    "toStampedTransformMsg",
    "toStampedPoseMsg"
  }

  return utils.create_method_table("tf_StampedTransform_", StampedTransform_method_names)
end

local f = init()

function StampedTransform:__init(transform, stamp, frame_id, child_frame_id)
  transform = transform or tf.Transform()
  if torch.isTensor(transform) then
    if transform:nDimension() == 2 and transform:size(1) == 4 and transform:size(2) == 4 then
      transform = tf.Transform():fromTensor(transform)
    else
      error('Invalid tensor specified. 4x4 matrix expected.')
    end
  end
  stamp = stamp or ros.Time.getNow()
  self.t = f.new(transform:cdata(), stamp:cdata(), frame_id or '', child_frame_id or '')
  self.o = f.getBasePointer(self.t)
  self.moveit_msgs_StampedTransform = ros.get_msgspec('geometry_msgs/TransformStamped')
  self.moveit_msgs_StampedPose = ros.get_msgspec('geometry_msgs/PoseStamped')
end

function StampedTransform:cdata()
  return self.t
end

function StampedTransform:clone()
  local c = torch.factory('tf.StampedTransform')()
  local _t = f.clone(self.t)
  rawset(c, 't', _t)
  rawset(c, 'o', f.getBasePointer(_t))
  return c
end

function StampedTransform:toTransform()
  return tf.Transform.fromStamped(self)
end

function StampedTransform:get_stamp(result)
  result = result or ros.Time()
  f.get_stamp(self.t, result:cdata())
  return result
end

function StampedTransform:set_stamp(stamp)
  f.set_stamp(self.t, stamp:cdata())
end

function StampedTransform:get_frame_id()
  return ffi.string(f.get_frame_id(self.t))
end

function StampedTransform:set_frame_id(frame_id)
  f.set_frame_id(self.t, frame_id)
end

function StampedTransform:get_child_frame_id()
  return ffi.string(f.get_child_frame_id(self.t))
end

function StampedTransform:set_child_frame_id(child_frame_id)
  f.set_child_frame_id(self.t, child_frame_id)
end

function StampedTransform:setData(transform)
  f.setData(self.t, transform:cdata())
end

function StampedTransform:toStampedTransformMsg(output)
  local msg_bytes = torch.ByteStorage()
  f.toStampedTransformMsg(self.t, msg_bytes:cdata())
  local msg = output or ros.Message(self.moveit_msgs_StampedTransform, true)
  msg:deserialize(msg_bytes)
  return msg
end

function StampedTransform:toStampedPoseMsg(output)
  local msg_bytes = torch.ByteStorage()
  f.toStampedPoseMsg(self.t, msg_bytes:cdata())
  local msg = output or ros.Message(self.moveit_msgs_StampedPose, true)
  msg:deserialize(msg_bytes)
  return msg
end

function StampedTransform:__eq(other)
  f.eq(self.t, other:cdata())
end

function StampedTransform:__tostring()
  local s = string.format('{\n  stamp: %s\n  frame_id: \'%s\'\n  child_frame_id: \'%s\'\n  transform:\n%s\n}',
    tostring(self:get_stamp()),
    self:get_frame_id(),
    self:get_child_frame_id(),
    parent.__tostring(self)
  )
  return s
end
