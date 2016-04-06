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
    'get_vector',
    'get_table',

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
    'set_string',
    'set_vector',
    'set_table'
  }

  return utils.create_method_table("std_Variable_", Variable_method_names)
end

local f = init()

local TYPE_CODE = {
  void = 0,
  bool = 1,
  int8 = 2,
  int16 = 3,
  int32 = 4,
  int64 = 5,
  uint8 = 6,
  uint16 = 7,
  uint32 = 8,
  uint64 = 9,
  float32 = 10,
  float64 = 11,
  string = 12,
  vector = 13,
  table = 14
}

local type_names = utils.reverse_mapping(TYPE_CODE, {})

Variable.TYPE_CODE = TYPE_CODE

function Variable:__init(x, type_code)
  self.o = f.new()

  if x ~= nil then
    self:set(x, type_code)
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

function Variable:get_type()
  return f.get_type(self.o)
end

function Variable:type()
  return type_names[self:get_type()]
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

function Variable:get_vector(value, result)
  result = result or std.VariableVector()
  f.get_vector(self.o, result:cdata())
  return result
end

function Variable:set_vector(value)
  if not torch.isTypeOf(value, std.VariableVector) then
    local vec = std.VariableVector()
    for i,v in ipairs(value) do
      vec:push_back(v)
    end
    value = vec
  end
  f.set_vector(self.o, value:cdata())
end

function Variable:get_table(value, result)
  result = result or std.VariableTable()
  f.get_table(self.o, result:cdata())
  return result
end

function Variable:set_table(value)
  if torch.isTypeOf(value, std.Variable) then
    self:set(value:get(), value:get_type())
  elseif torch.isTypeOf(value, std.VariableVector) then
    self:set_vector(value)
  elseif torch.isTypeOf(value, std.VariableTable) then
    f.set_table(self.o, value:cdata())
  elseif value and #value > 0 then
    value = std.VariableVector(value)
    f.set_vector(self.o, value:cdata())
  else
    value = std.VariableTable(value)
    f.set_table(self.o, value:cdata())
  end
end

local function create_getter_table()
  local t = {}
  t[TYPE_CODE.void]     = function(self) return nil end
  t[TYPE_CODE.bool]     = Variable.get_bool
  t[TYPE_CODE.int8]     = Variable.get_int8
  t[TYPE_CODE.int16]    = Variable.get_int16
  t[TYPE_CODE.int32]    = Variable.get_int32
  t[TYPE_CODE.int64]    = Variable.get_int64
  t[TYPE_CODE.uint8]    = Variable.get_uint8
  t[TYPE_CODE.uint16]   = Variable.get_uint16
  t[TYPE_CODE.uint32]   = Variable.get_uint32
  t[TYPE_CODE.uint64]   = Variable.get_uint64
  t[TYPE_CODE.float32]  = Variable.get_float32
  t[TYPE_CODE.float64]  = Variable.get_float64
  t[TYPE_CODE.string]   = Variable.get_string
  t[TYPE_CODE.vector]   = Variable.get_vector
  t[TYPE_CODE.table]    = Variable.get_table
  return t
end

local getter_table = create_getter_table()

function Variable:get()
  local t = self:get_type()
  if t == TYPE_CODE.vector then
    return self:get_vector():totable()
  elseif t == TYPE_CODE.table then
    return self:get_table():totable()
  else
    return getter_table[self:get_type()](self)
  end
end

local function create_setter_table()
  local t = {}
  t[TYPE_CODE.void]     = function(self, value) self:clear() end
  t[TYPE_CODE.bool]     = Variable.set_bool
  t[TYPE_CODE.int8]     = Variable.set_int8
  t[TYPE_CODE.int16]    = Variable.set_int16
  t[TYPE_CODE.int32]    = Variable.set_int32
  t[TYPE_CODE.int64]    = Variable.set_int64
  t[TYPE_CODE.uint8]    = Variable.set_uint8
  t[TYPE_CODE.uint16]   = Variable.set_uint16
  t[TYPE_CODE.uint32]   = Variable.set_uint32
  t[TYPE_CODE.uint64]   = Variable.set_uint64
  t[TYPE_CODE.float32]  = Variable.set_float32
  t[TYPE_CODE.float64]  = Variable.set_float64
  t[TYPE_CODE.string]   = Variable.set_string
  t[TYPE_CODE.vector]   = Variable.set_vector
  t[TYPE_CODE.table]    = Variable.set_table
  return t
end

local setter_table = create_setter_table()

function Variable:set(value, type_code)
  if value == nil then
    self:clear()
  else
    if type_code ~= nil and type(type_code) == 'string' then
      type_code = TYPE_CODE[string.lower(type_code)]
    end

    local setter
    if type(type_code) == 'number' then
      setter = setter_table[type_code]
      if setter == nil then
        error('Unsupported type_code specified.')
      end
    end

    if setter then
      setter(self, value)
    else
      local t = type(value)
      if t == 'boolean' then
        self:set_bool(value)
      elseif t == 'number' then
        self:set_float64(value)
      elseif t == 'string' then
        self:set_string(value)
      elseif t == 'table' then
        self:set_table(value)
      else
        self:set_string(tostring(value))
      end
    end
  end
end

function Variable:__tostring()
  return tostring(self:get()) .. ' [' .. self:type() .. ']'
end
