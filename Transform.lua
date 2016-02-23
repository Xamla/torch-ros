local ffi = require 'ffi'
local torch = require 'torch'
local ros = require 'ros.env'
local utils = require 'ros.utils'
local tf = ros.tf

local Transform = torch.class('tf.Transform', tf)

local f

function init()
  local Transform_method_names = {
    "new",
    "clone",
    "delete",
    "setIdentity",
    "mul_Quaternion",
    "mul_Transform",
    "inverse",
    "getBasis",
    "getOrigin",
    "setRotation",
    "getRotation",
  }
  
  f = utils.create_method_table("tf_Transform_", Transform_method_names )
end

init()

function Transform:__init()
  self.o = f.new()
  self:setIdentity()
end

function Transform:cdata()
  return self.o
end

function Transform:clone()
  local c = torch.factory('tf.Transform')()
  rawset(c, 'o', f.clone(self.o))
  return c
end

function Transform:setIdentity()
  f.setIdentity(self.o)
end

function Transform:getBasis(basis)
  basis = base or torch.DoubleTensor()
  f.getBasis(self.o, basis:cdata())
  return basis
end

function Transform:setBasis(basis)
  self:getBasis()[{}] = basis
end

function Transform:mul(t, output)
  output = output or tf.Transform()
  if torch.isTypeOf(t, tf.Transform) then
    f.mul_Transform(self.o, t:cdata(), output:cdata())
  elseif torch.isTypeOf(t, tf.Quaternion) then
    f.mul_Quaternion(self.o, t:cdata(), output:cdata())
  else
    error('tf.Transform or tf.Quaternion expected')
  end
  return output
end

function Transform:getOrigin(origin)
  origin = origin or torch.DoubleTensor()
  f.getOrigin(self.o, origin:cdata())
  return origin
end

function Transform:setOrigin(origin)
  if not torch.isTensor(origin) then
    origin = torch.DoubleTensor(origin)
  end
  self:getOrigin()[{}] = origin
end

function Transform:inverse(output)
  output = output or tf.Transform()
  f.inverse(self.o, output:cdata())
  return output
end

function Transform:toTensor()
  local t = torch.zeros(4,4)
  t[{{1,3},{1,3}}] = self:getBasis()
  t[{{1,3},{4}}] = self:getOrigin()
  t[{4,4}] = 1
  return t
end

function Transform:fromTensor(t)
  self:getBasis()[{}] = t[{{1,3},{1,3}}]
  self:getOrigin()[{}] = t[{{4},{1,3}}]
end

function Transform:getRotation(output)
  output = output or tf.Quaternion()
  f.getRotation(self.o, output:cdata())
  return output
end

function Transform:setRotation(quaternion)
  f.setRotation(self.o, quaternion:cdata())
end

function Transform:__tostring()
  local t = self:toTensor()
  local s = ''
  for i=1,4 do
    s = s .. string.format('%9g %9g %9g %9g\n', t[{i,1}], t[{i,2}], t[{i,3}], t[{i,4}])
  end
  return s
end
