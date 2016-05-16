local ffi = require 'ffi'
local torch = require 'torch'
local ros = require 'ros.env'
local utils = require 'ros.utils'

local Duration = torch.class('ros.Duration', ros)

function init()
  local Duration_method_names = {
    "new",
    "clone",
    "delete",
    "set",
    "assign",
    "get_sec",
    "set_sec",
    "get_nsec",
    "set_nsec",
    "add",
    "sub",
    "mul",
    "eq",
    "lt",
    "toSec",
    "fromSec",
    "fromNSec",
    "isZero",
    "sleep"
  }

  return utils.create_method_table("ros_Duration_", Duration_method_names)
end

local f = init()

function Duration:__init(_1, _2)
  self.o = f.new()
  if _1 or _2 then
    self:set(_1, _2)
  end
end

function Duration:cdata()
  return self.o
end

function Duration:clone()
  local c = torch.factory('ros.Duration')()
  rawset(c, 'o', f.clone(self.o))
  return c
end

function Duration:set(_1, _2)
  if torch.isTypeOf(_1, ros.Duration) then
    self:assign(_1)
  elseif not _2 then
    self:fromSec(_1)
  else
    f.set(self.o, _1, _2)
  end
end

function Duration:get()
  return self:get_sec(), self:get_nsec()
end

function Duration:assign(other)
  f.assign(self.o, other:cdata())
end

function Duration:get_sec()
  return f.get_sec(self.o)
end

function Duration:set_sec(sec)
  f.set_sec(self.o, sec)
end

function Duration:get_nsec()
  return f.get_nsec(self.o)
end

function Duration:set_nsec(nsec)
  f.set_nesc(self.o, nsec)
end

function Duration:add(other, result)
  result = result or self
  if type(other) == 'number' then
    other = ros.Duration(other)
  end
  f.add(self.o, other:cdata(), result:cdata())
  return result
end

function Duration:sub(other, result)
  result = result or self
  if type(other) == 'number' then
    other = ros.Duration(other)
  end
  f.sub(self.o, other:cdata(), result:cdata())
  return result
end

function Duration:mul(factor, result)
  result = result or self
  f.mul(self.o, factor, result:cdata())
  return result
end

function Duration:__mul(f)
  local result = ros.Duration()
  return self:mul(f, result)
end

function Duration:__sub(d)
  local result = ros.Duration()
  return self:sub(d, result)
end

function Duration:__add(d)
  local result = ros.Duration()
  return self:add(d, result)
end

function Duration:__eq(other)
  return self ~= nil and other ~=nil and f.eq(self.o, other:cdata())
end

function Duration:__lt(other)
  return self ~= nil and other ~= nil and f.lt(self.o, other:cdata())
end

function Duration:__le(other)
  return self ~= nil and other ~= nil and (f.lt(self.o, other:cdata()) or f.eq(self.o, other:cdata()))
end

function Duration:toSec()
  return f.toSec(self.o)
end

function Duration:fromSec(sec)
  f.fromSec(self.o, sec)
end

function Duration:isZero()
  return f.isZero(self.o, sec)
end

function Duration:sleep()
  f.sleep(self.o)
end

function Duration:__tostring()
  return string.format("%f", self:toSec())
end
