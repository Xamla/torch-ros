local ffi = require 'ffi'
local torch = require 'torch'
local ros = require 'ros.env'
local utils = require 'ros.utils'
std = ros.std

local VariableTable = torch.class('std.VariableTable', std)

function init()
  local VariableTable_method_names = {
    'new',
    'clone',
    'delete',
    'size',
    'clear',
    'getField',
    'setField',
    'erase',
    'exists',
    'keys',
    'values'
  }

  return utils.create_method_table("std_VariableTable_", VariableTable_method_names)
end

local f = init()

function VariableTable:__init(x)
  rawset(self, 'o', f.new())
  if x ~= nil and type(x) == 'table' then
    self:insertFromTable(x)
  end
end

function VariableTable:cdata()
  return rawget(self, 'o')
end

function VariableTable:clone()
  local c = torch.factory('std.VariableTable')()
  rawset(c, 'o', f.clone(self.o))
  return c
end

function VariableTable:size()
  return f.size(self.o)
end

function VariableTable:__len()
  return self:size()
end

function VariableTable:erase(key)
  f.erase(self.o, key)
end

function VariableTable:insertFromTable(t)
  for k,v in pairs(t) do
    if not torch.isTypeOf(v, std.Variable) then
      v = std.Variable(v)
    end
    self:setField(k, v)
  end
end

function VariableTable:getField(key)
  local o = rawget(self, 'o')
  local v = std.Variable()
  if not f.getField(self.o, key, v:cdata()) then
    return nil
  end
  return v
end

function VariableTable:setField(key, value)
  local o = rawget(self, 'o')
  if not torch.isTypeOf(value, std.Variable) then
    value = std.Variable(value)
  end
  f.setField(o, key, value:cdata())
end

function VariableTable:__index(key)
  local v = rawget(self, key)
  if not v then
    v = VariableTable[key]
    if not v and type(key) == 'string' then
      v = self:getField(key)
    end
  end
  return v
end

function VariableTable:__newindex(key, value)
  local o = rawget(self, 'o')
  if type(key) == 'string' then
    self:setField(key, value)
  else
    rawset(self, key, value)
  end
end

function VariableTable:exists(key)
  return f.exists(self.o, key)
end

function VariableTable:keys()
  local v = std.StringVector()
  f.keys(self.o, v:cdata())
  return v
end

function VariableTable:values()
  local v = std.VariableVector()
  f.values(self.o, v:cdata())
  return v
end

function VariableTable:totable()
  local k,v = self:keys(),self:values()
  local r = {}
  for i=1,#k do
    r[k[i]] = v[i]:get()
  end
  return r
end

function VariableTable:__tostring()
  local t = {}
  table.insert(t, '{')
  local k,v = self:keys(),self:values()
  for i=1,#k do
    table.insert(t, '  ' .. k[i] .. ' : ' .. tostring(v[i]))
  end
  table.insert(t, '}')
  table.insert(t, string.format('[%s]', torch.type(self)))
  return table.concat(t, '\n')
end
