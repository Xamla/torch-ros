local ffi = require 'ffi'
local torch = require 'torch'
local ros = require 'ros.env'
local utils = require 'ros.utils'
std = ros.std

local Variable = torch.class('std.Variable', std)

function init()
  local Variable_method_names = {
    'new',
    'clone',
    'delete',
    'get_type',
    'clear',
    'assign',

    'get_bool',
    'get_int8',
    'get_int16',
    'get_int32',
    'get_int64',
    'get_uint8',
    'get_uint16',
    'get_uint32',
    'get_uint64',
    'get_float32',
    'get_float64',
    'get_string',

    'set_bool',
    'set_int8',
    'set_int16',
    'set_int32',
    'set_int64',
    'set_uint8',
    'set_uint16',
    'set_uint32',
    'set_uint64',
    'set_float32',
    'set_float64',
    'set_string'
  }

  return utils.create_method_table("std_Variable_", Variable_method_names)
end

local f = init()

local TYPE_CODE = {
  Void = 0,
  Bool = 1,
  Int8 = 2,
  Int16 = 3,
  Int32 = 4,
  Int64 = 5,
  UInt8 = 6,
  UInt16 = 7,
  UInt32 = 8,
  UInt64 = 9,
  Float32 = 10,
  Float64 = 11,
  String = 12
}

Variable.TYPE_CODE = TYPE_CODE

function Variable:__init(x)
  self.o = f.new()

  if x then
    self:set(x)
  end
end

function Variable:clone()
  local c = torch.factory('std.Variable')()
  rawset(c, 'o', f.clone(self.o))
  return c
end

function Variable:cdata()
  return self.o
end

function Variable:type()
  return f.get_type(self.o)
end

function Variable:clear()
  f.clear(self.o)
end

function Variable:assign(src)
  f.assign(self.o, src:cdata())
end

local function create_getters()
  local names = {
    'get_bool',
    'get_int8',
    'get_int16',
    'get_int32',
    'get_int64',
    'get_uint8',
    'get_uint16',
    'get_uint32',
    'get_uint64',
    'get_float32',
    'get_float64'
  }
  for i,name in ipairs(names) do
    local g = f[name];
    Variable[name] = function(self)
      return g(self.o);
    end
  end
end
create_getters()

local function create_setters()
  local names = {
    'set_bool',
    'set_int8',
    'set_int16',
    'set_int32',
    'set_int64',
    'set_uint8',
    'set_uint16',
    'set_uint32',
    'set_uint64',
    'set_float32',
    'set_float64',
    'set_string'
  }
  for i,name in ipairs(names) do
    local s = f[name];
    Variable[name] = function(self, value)
      s(self.o, value)
    end
  end
end
create_setters()

function Variable:get_string()
  return ffi.string(f.get_string(self.o))
end

function create_getter_table()
  local t = {}
  t[TYPE_CODE.Void] = function(self) return nil end
  t[TYPE_CODE.Bool] = Variable.get_bool
  t[TYPE_CODE.Int8] = Variable.get_int8
  t[TYPE_CODE.Int16] = Variable.get_int16
  t[TYPE_CODE.Int32] = Variable.get_int32
  t[TYPE_CODE.Int64] = Variable.get_int64
  t[TYPE_CODE.UInt8] = Variable.get_uint8
  t[TYPE_CODE.UInt16] = Variable.get_uint16
  t[TYPE_CODE.UInt32] = Variable.get_uint32
  t[TYPE_CODE.UInt64] = Variable.get_uint64
  t[TYPE_CODE.Float32] = Variable.get_float32
  t[TYPE_CODE.Float64] = Variable.get_float64
  t[TYPE_CODE.String] = Variable.get_string
  return t
end

local getter_table = create_getter_table()

function Variable:get()
  return getter_table[self:type()](self)
end

function Variable:set(value)
  if not value then
    self:clear()
  else
    local t = type(value)
    if t == 'boolean' then
      self:set_bool(value)
    elseif t == 'number' then
      self:set_float64(value)
    elseif t == 'string' then
      self:set_string(value)
    else
      self:set_string(tostring(value))
    end
  end
end

function Variable:__tostring()
  return tostring(self:get())
end
