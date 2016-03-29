local ffi = require 'ffi'
local torch = require 'torch'
local ros = require 'ros.env'
local utils = require 'ros.utils'
local std = ros.std

local StringMap = torch.class('std.StringMap', std)

function init()
  local StringMap_method_names = {
    "new",
    "clone",
    "delete",
    "size",
    "clear",
    "getAt",
    "setAt",
    "insert",
    "erase",
    "exists",
    "keys",
    "values"
  }

  return utils.create_method_table("std_StringMap_", StringMap_method_names)
end

local f = init()

function StringMap:__init(x)
  rawset(self, 'o', f.new())
  if x ~= nil and type(x) == 'table' then
    self:insertFromTable(x)
  end
end

function StringMap:cdata()
  return self.o
end

function StringMap:clone()
  local c = torch.factory('std.StringMap')()
  rawset(c, 'o', f.clone(self.o))
  return c
end

function StringMap:size()
  return f.size(self.o)
end

function StringMap:__len()
  return self:size()
end

function StringMap:clear()
  f.clear(self.o)
end

function StringMap:insert(key, value)
  local o = rawget(self, 'o')
  return f.insert(o, key, value)
end

function StringMap:insertFromTable(t)
  for k,v in pairs(t) do
    self:setAt(k, tostring(v))
  end
end

function StringMap:erase(key)
  return f.erase(self.o, key)
end

function StringMap:getAt(key)
  return ffi.string(f.getAt(self.o, key))
end

function StringMap:setAt(key, value)
  f.setAt(self.o, key, value)
end

function StringMap:__index(key)
  local v = rawget(self, key)
  if not v then
    v = StringMap[key]
    if not v and type(key) == 'string' then
      v = self:getAt(key)
    end
  end
  return v
end

function StringMap:__newindex(key, value)
  local o = rawget(self, 'o')
  if type(key) == 'string' then
    self:setAt(key, value)
  else
    rawset(self, key, value)
  end
end

function StringMap:keys()
  local v = std.StringVector()
  f.keys(self.o, v:cdata())
  return v
endvalues

function StringMap:values()
  local v = std.StringVector()
  f.values(self.o, v:cdata())
  return v
end

function StringMap:totable()
  local k,v = self:keys(),self:values()
  local r = {}
  for i=1,#k do
    r[k[i]] = v[i]
  end
  return r
end

function StringMap:__tostring()
  local t = {}
  table.insert(t, '{')
  local k,v = self:keys(),self:values()
  for i=1,#k do
    table.insert(t, '  ' .. k[i] .. ' : ' .. v[i])
  end
  table.insert(t, '}')
  table.insert(t, string.format('[%s]', torch.type(self)))
  return table.concat(t, '\n')
end
