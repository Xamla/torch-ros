local ffi = require 'ffi'
local torch = require 'torch'
local ros = require 'ros.env'
local utils = require 'ros.utils'
std = ros.std

local VariableMap = torch.class('std.VariableMap', std)

function init()
  local VariableMap_method_names = {
    'new',
    'clone',
    'delete',
    'size',
    'clear',
    'getAt',
    'setAt',
    'erase',
    'exists',
    'keys',
    'values'
  }

  return utils.create_method_table("std_VariableMap_", VariableMap_method_names)
end

local f = init()

function VariableMap:__init(x)
  rawset(self, 'o', f.new())
  if x and type(x) == 'table' then
    self:insertFromTable(x)
  end
end

function VariableMap:cdata()
  return rawget(self, 'o')
end

function VariableMap:clone()
  local c = torch.factory('std.VariableMap')()
  rawset(c, 'o', f.clone(self.o))
  return c
end

function VariableMap:size()
  return f.size(self.o)
end

function VariableMap:__len()
  return self:size()
end

function VariableMap:erase(key)
  f.erase(self.o, key)
end

function VariableMap:insertFromTable(t)
  for k,v in pairs(t) do
    if not torch.isTypeOf(v, std.Variable) then
      v = std.Variable(v)
    end
    self:setAt(k, v)
  end
end

function VariableMap:getAt(key)
  local o = rawget(self, 'o')
  local v = std.Variable()
  if not f.getAt(self.o, key, v:cdata()) then
    return nil
  end
  return v
end

function VariableMap:setAt(key, value)
  local o = rawget(self, 'o')
  if not torch.isTypeOf(value, std.Variable) then
    value = std.Variable(value)
  end
  f.setAt(o, key, value:cdata())
end

function VariableMap:__index(key)
  local v = rawget(self, key)
  if not v then
    v = VariableMap[key]
    if not v and type(key) == 'string' then
      v = self:getAt(key)
    end
  end
  return v
end

function VariableMap:__newindex(key, v)
  local o = rawget(self, 'o')
  if type(key) == 'string' then
    self:setAt(key, value)
  else
    rawset(self, key, v)
  end
end

function VariableMap:exists(key)
  return f.exists(self.o, key)
end

function VariableMap:keys()
  local v = std.StringVector()
  f.keys(self.o, v:cdata())
  return v
end

function VariableMap:values()
  local v = std.VariableVector()
  f.values(self.o, v:cdata())
  return v
end

function VariableMap:totable()
  local k = self:keys()
  local v = self:values()
  local r = {}
  for i=1,#k do
    r[k[i]] = v[i]:get()
  end
  return r
end
