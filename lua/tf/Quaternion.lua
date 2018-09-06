local ffi = require 'ffi'
local torch = require 'torch'
local ros = require 'ros.env'
local utils = require 'ros.utils'
local tf = ros.tf

local Quaternion = torch.class('tf.Quaternion', tf)

function init()
  local Quaternion_method_names = {
    "new",
    "clone",
    "delete",
    "setIdentity",
    "setRotation_Tensor",
    "setEuler",
    "getRPY",
    "setRPY",
    "getAngle",
    "getAxis_Tensor",
    "inverse",
    "length2",
    "normalize",
    "angle",
    "angleShortestPath",
    "add",
    "sub",
    "mul",
    "mul_scalar",
    "div_scalar",
    "dot",
    "slerp",
    "viewTensor"
  }

  return utils.create_method_table("tf_Quaternion_", Quaternion_method_names )
end

local f = init()

function Quaternion:__init(_1, _2, _3, _4)
  self.o = f.new()
  if _1 then
    if type(_1) == 'table' then
      _1 = torch.DoubleTensor(_1)
    end
    if torch.isTensor(_1) then
      if type(_2) == 'number' then
        self:setRotation(_1, _2, _3)
      else
        self:fromTensor(_1)
      end
    elseif torch.isTypeOf(x, tf.Quaternion) then
      self:fromTensor(x:toTensor())
    elseif _2 and _3 and _4 then
      self:fromTensor(torch.DoubleTensor({_1, _2, _3, _4}))
    else
      error('Invalid arguments for tf.Quaternion ctor')
    end
  else
    self:setIdentity()
  end
end

function Quaternion:cdata()
  return self.o
end

function Quaternion:clone()
  local c = torch.factory('tf.Quaternion')()
  rawset(c, 'o', f.clone(self.o))
  return c
end

function Quaternion:setIdentity()
  f.setIdentity(self.o)
end

function Quaternion:setRotation(axis, angle, deg)
  if deg then
    angle = math.rad(angle)
  end
  if type(axis) == 'table' then
    axis = torch.DoubleTensor(axis)
  end
  if torch.isTensor(axis) then
    f.setRotation_Tensor(self.o, axis:cdata(), angle)
  else
    error('Tensor axis expected')
  end
  return self
end

function Quaternion:setEuler(yaw, pitch, roll, deg)
  if deg then
    roll, pitch, yaw = math.rad(roll), math.rad(pitch), math.rad(yaw)
  end
  f.setEuler(self.o, yaw, pitch, roll)
  return self
end

function Quaternion:getRPY(solution_number)
  local output = torch.DoubleTensor()
  f.getRPY(self.o, solution_number or 1, output:cdata())
  return output
end

function Quaternion:setRPY(roll, pitch, yaw, deg)
  if deg then
    roll, pitch, yaw = math.rad(roll), math.rad(pitch), math.rad(yaw)
  end
  if type(roll) == 'number' then
    f.setRPY(self.o, roll, pitch, yaw)
  else
    f.setRPY(self.o, roll[1],roll[2],roll[3])
  end
  return self
end

function Quaternion:getAngle()
  return f.getAngle(self.o)
end

function Quaternion:getAxis(axis)
  axis = axis or torch.DoubleTensor()
  f.getAxis_Tensor(self.o, axis:cdata())
  return axis
end

function Quaternion:inverse()
  result = tf.Quaternion()
  f.inverse(self.o, result:cdata())
  return result
end

function Quaternion:length2()
  return f.length2(self.o)
end

function Quaternion:length()
  return math.sqrt(self:length2())
end

function Quaternion:normalize()
  f.normalize(self.o)
  return self
end

function Quaternion:normalized()
  local c = self:clone()
  c:normalize()
  return c
end

function Quaternion:angle(other)
  return f.angle(self.o, other:cdata())
end

function Quaternion:angleShortestPath(other)
  assert(torch.isTypeOf(other, tf.Quaternion),"Invalid argument")
  return f.angleShortestPath(self.o, other:cdata())
end

function Quaternion:add(other, result)
  local result = result or tf.Quaternion()
  f.add(self.o, other:cdata(), result:cdata())
  return result
end

function Quaternion:sub(other, result)
  local result = result or tf.Quaternion()
  f.sub(self.o, other:cdata(), result:cdata())
  return result
end

function Quaternion:mul(other, result)
  local result = result or tf.Quaternion()
  if torch.isTypeOf(other, tf.Quaternion) then
    f.mul(self.o, other:cdata(), result:cdata())
  elseif type(other) == 'number' then
    f.mul_scalar(self.o, other, result:cdata())
  else
    error('Unsupported type of factor for quaternion multiplication.')
  end
  return result
end

function Quaternion:__mul(other)
  return self:mul(other)
end

function Quaternion:div(divisor, result)
  result = result or tf.Quaternion()
  if type(divisor) == 'number' then
    f.div_scalar(self.o, divisor, result:cdata())
  else
    error('Unsupported type of factor for quaternion division.')
  end
  return result
end

function Quaternion:dot(other)
  return f.dot(self.o, other:cdata())
end

function Quaternion:slerp(other, t, result)
  result = result or tf.Quaternion()
  f.slerp(self.o, other:cdata(), t, result:cdata())
  return result
end

function Quaternion:fromTensor(t)
  local v = self:toTensor()
  v[{}] = t
end

function Quaternion:toTensor()
  local result = torch.DoubleTensor()
  f.viewTensor(self.o, result:cdata())
  return result
end

function Quaternion:toMatrixTensor()
  local result = torch.DoubleTensor(3,3)
  local q = self:toTensor()
  local sqw = q[4]*q[4]
  local sqx = q[1]*q[1]
  local sqy = q[2]*q[2]
  local sqz = q[3]*q[3]

  local invs = 1 / (sqx + sqy + sqz + sqw)
  result[1][1] = ( sqx - sqy - sqz + sqw)*invs
  result[2][2] = (-sqx + sqy - sqz + sqw)*invs
  result[3][3] = (-sqx - sqy + sqz + sqw)*invs

  local tmp1 = q[1]*q[2];
  local tmp2 = q[3]*q[4];
  result[2][1] = 2.0 * (tmp1 + tmp2)*invs
  result[1][2] = 2.0 * (tmp1 - tmp2)*invs

  tmp1 = q[1]*q[3]
  tmp2 = q[2]*q[4]
  result[3][1] = 2.0 * (tmp1 - tmp2)*invs
  result[1][3] = 2.0 * (tmp1 + tmp2)*invs
  tmp1 = q[2]*q[3]
  tmp2 = q[1]*q[4]
  result[3][2] = 2.0 * (tmp1 + tmp2)*invs
  result[2][3] = 2.0 * (tmp1 - tmp2)*invs
  return result
end

function Quaternion:conjugate()
  local c = self:clone()
  local qt = self:toTensor():clone()
  qt[{{1,3}}]:mul(-1)
  c:fromTensor(qt)
  return c
end

function Quaternion:getAxisAngle()
  local axis = self:getAxis()
  local angle = self:getAngle()
  local norm = axis:norm()
  if (norm > 1e-5) then
    axis = axis * (angle / norm)
  else
    axis:zero()
  end
  return axis
end

function Quaternion:setAxisAngle(value)
    local c = self:clone()
    local norm = value:norm()
    local w = math.cos(0.5 * norm)
    local sin_a = math.sin(0.5 * norm)
    local result = c:toTensor()
    result[4] = w
    if (norm > 1e-5) then
      result[{{1,3}}] = value * sin_a / norm
    else
      result[{{1,3}}]:zero()
    end
    return c
end

function Quaternion:__tostring()
  local t = self:toTensor()
  return string.format("{ x:%f, y:%f, z:%f, w:%f }", t[1], t[2], t[3], t[4]);
end
