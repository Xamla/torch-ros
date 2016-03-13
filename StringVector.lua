local ffi = require 'ffi'
local torch = require 'torch'
local ros = require 'ros.env'
local utils = require 'ros.utils'
local std = ros.std

local StringVector = torch.class('std.StringVector', std)

function init()
  local StringVector_method_names = {
    "new",
    "clone",
    "delete",
    "size",
    "getAt",
    "setAt",
    "push_back",
    "pop_back",
    "clear",
    "insert",
    "erase",
    "empty"
  }
  
  return utils.create_method_table("std_StringVector_", StringVector_method_names)
end

local f = init()

function StringVector:__init(...)
  rawset(self, 'o', f.new())
  if select("#", ...) > 0 then
    local x = ...
    if type(x) ~= 'table' then
      x = { ... }
    end
    self:insertFromTable(1, x)
  end
end

function StringVector:cdata()
  return self.o
end

function StringVector:clone()
  local c = torch.factory('std.StringVector')()
  rawset(c, 'o', f.clone(self.o))
  return c
end

function StringVector:size()
  return f.size(self.o)
end

function StringVector:__len()
  return self:size()
end

function StringVector:__index(idx)
  local v = rawget(self, idx)
  if not v then 
    v = StringVector[idx]
    if not v and type(idx) == 'number' then
      local o = rawget(self, 'o')
      v = ffi.string(f.getAt(o, idx-1))
    end
  end
  return v
end

function StringVector:__newindex(idx, v)
  local o = rawget(self, 'o')
  if type(idx) == 'number' then
    f.setAt(o, idx-1, tostring(v))
  else
    rawset(self, idx, v)
  end
end

function StringVector:push_back(value)
  f.push_back(self.o, tostring(value))
end

function StringVector:pop_back()
  local last = self[#self]
  f.pop_back(self.o)
  return last
end

function StringVector:clear()
  f.clear(self.o)
end

function StringVector:insert(pos, value, n)
  if pos < 1 then
    pos = 1
  elseif pos > #self+1 then
    pos = #self + 1
  end
  f.insert(self.o, pos-1, n or 1, value)
end

function StringVector:insertFromTable(pos, t)
  if type(pos) == 'table' then
    t = pos
    pos = #self + 1
  end
  pos = pos or #self + 1
  for _,v in pairs(t) do
    self:insert(pos, v)
    pos = pos + 1
  end
end

function StringVector:erase(begin_pos, end_pos)
  f.erase(self.o, begin_pos-1, (end_pos or begin_pos + 1)-1)
end

function StringVector:__pairs()
  return function (t, k)
    local i = k or 1
    if i > #t then
      return nil
    else
      local v = t[i]
      return i+1, v
    end
  end, self, nil
end

function StringVector:__ipairs()
  return self:__pairs()
end

function StringVector:totable()
  local t = {}
  for i,v in ipairs(self) do
    table.insert(t, v)
  end
  return t
end

function StringVector:__tostring()
  local t = self:totable()
  return table.concat(t, '\n')
end
