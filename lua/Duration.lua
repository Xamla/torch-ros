--- ROS data class to handle time intervals and durations
-- @classmod Duration

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

--- Construct a time duration
-- @tparam ?number|ros.Duration _1 If number: time duration in secons, fractional number possible
-- @tparam[opt] number _2 If present, _1 represends the seconds and _2 represends the nanoseconds
function Duration:__init(_1, _2)
  self.o = f.new()
  if _1 or _2 then
    self:set(_1, _2)
  end
end

--- Get the underlying data structure
-- @return
function Duration:cdata()
  return self.o
end

--- Creates a deep copy of the object
-- @return A copy of the object
function Duration:clone()
  local c = torch.factory('ros.Duration')()
  rawset(c, 'o', f.clone(self.o))
  return c
end

--- Set the time value
-- @tparam ?number|ros.Duration _1 If number: time duration in secons, fractional number possible
-- @tparam[opt] number _2 If present, _1 represends the seconds and _2 represends the nanoseconds
function Duration:set(_1, _2)
  if torch.isTypeOf(_1, ros.Duration) then
    self:assign(_1)
  elseif not _2 then
    self:fromSec(_1)
  else
    f.set(self.o, _1, _2)
  end
end

--- Get the time value as seconds and nanoseconds
-- @treturn number seconds
-- @treturn number nanoseconds
function Duration:get()
  return self:get_sec(), self:get_nsec()
end

--- Assign operation
-- @tparam ros.Duration other Instance to set the values from
function Duration:assign(other)
  f.assign(self.o, other:cdata())
end

--- Get the seconds part of the stored duration
-- @treturn number seconds
function Duration:get_sec()
  return f.get_sec(self.o)
end

--- Set duration in seconds
-- @tparam number sec seconds
function Duration:set_sec(sec)
  f.set_sec(self.o, sec)
end

--- Get the nanoseconds part of the stored duration
-- @treturn number nanoseconds
function Duration:get_nsec()
  return f.get_nsec(self.o)
end

--- Set the nanoseconds part of the duration
-- @tparam number nsec nanoseconds
function Duration:set_nsec(nsec)
  f.set_nsec(self.o, nsec)
end

--- Add two durations and return the result
-- @tparam ?number|ros:Duration other The value to add, either the number of seconds (fractional numbers possible) or a ros:Duration
-- @tparam[opt] ros:Duration result If presend, the sum is assigned to result. Otherwise the operation is performed inplace.
function Duration:add(other, result)
  result = result or self
  if type(other) == 'number' then
    other = ros.Duration(other)
  end
  f.add(self.o, other:cdata(), result:cdata())
  return result
end

--- Substract a duration from this and return the result
-- @tparam ?number|ros:Duration other The value to substract, either the number of seconds (fractional numbers possible) or a ros:Duration
-- @tparam[opt] ros:Duration result If presend, the difference is assigned to result. Otherwise the operation is performed inplace.
function Duration:sub(other, result)
  result = result or self
  if type(other) == 'number' then
    other = ros.Duration(other)
  end
  f.sub(self.o, other:cdata(), result:cdata())
  return result
end

--- Multiply a duration to this and return the result
-- @tparam ?number|ros:Duration factor The value to multiply with, either the number of seconds (fractional numbers possible) or a ros:Duration
-- @tparam[opt] ros:Duration result If presend, the product is assigned to result. Otherwise the operation is performed inplace.
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

--- Converts the stored duration to seconds with fractional part
-- @treturn number Number of seconds
function Duration:toSec()
  return f.toSec(self.o)
end

--- Stores a (fractional) number of seconds in this data structure
-- @tparam number sec Number of seconds
function Duration:fromSec(sec)
  f.fromSec(self.o, sec)
end

--- Check if the Duration is zero
-- @treturn bool
function Duration:isZero()
  return f.isZero(self.o, sec)
end

--- Sleep for the (fractional) number of seconds stored in this data structure
function Duration:sleep()
  f.sleep(self.o)
end

function Duration:__tostring()
  return string.format("%f", self:toSec())
end
