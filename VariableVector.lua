local ffi = require 'ffi'
local torch = require 'torch'
local ros = require 'ros.env'
local utils = require 'ros.utils'
std = ros.std

local VariableVector = torch.class('std.VariableVector', std)

function init()
  local VariableVector_method_names = {
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

  return utils.create_method_table("std_VariableVector_", VariableVector_method_names)
end

local f = init()

function VariableVector:__init(...)
  rawset(self, 'o', f.new())
  if select("#", ...) > 0 then
    local x = ...
    if type(x) ~= 'table' then
      x = { ... }
    end
    self:insertFromTable(1, x)
  end
end

function VariableVector:cdata()
  return self.o
end

function VariableVector:clone()
  local c = torch.factory('std.VariableVector')()
  rawset(c, 'o', f.clone(self.o))
  return c
end

function VariableVector:size()
  return f.size(self.o)
end

function VariableVector:__len()
  return self:size()
end

function VariableVector:__index(idx)
  local v = rawget(self, idx)
  if not v then
    v = VariableVector[idx]
    if not v and type(idx) == 'number' then
      local o = rawget(self, 'o')
      v = std.Variable()
      f.getAt(o, idx-1, v:cdata())
    end
  end
  return v
end

function VariableVector:__newindex(idx, v)
  local o = rawget(self, 'o')
  if type(idx) == 'number' then
    if not torch.isTypeOf(v, std.Variable) then
      v = std.Variable(v)
    end
    f.setAt(o, idx-1, v)
  else
    rawset(self, idx, v)
  end
end

function VariableVector:push_back(value)
  if not torch.isTypeOf(value, std.Variable) then
    value = std.Variable(value)
  end
  f.push_back(self.o, value:cdata())
end

function VariableVector:pop_back()
  local last = self[#self]
  f.pop_back(self.o)
  return last
end

function VariableVector:clear()
  f.clear(self.o)
end

function VariableVector:insert(pos, value, n)
  if not torch.isTypeOf(value, std.Variable) then
    value = std.Variable(value)
  end
  if pos < 1 then
    pos = 1
  elseif pos > #self+1 then
    pos = #self + 1
  end
  f.insert(self.o, pos-1, n or 1, value:cdata())
end

function VariableVector:insertFromTable(pos, t)
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

function VariableVector:erase(begin_pos, end_pos)
  f.erase(self.o, begin_pos-1, (end_pos or begin_pos + 1)-1)
end

function VariableVector:__pairs()
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

function VariableVector:__ipairs()
  return self:__pairs()
end

function VariableVector:totable()
  local t = {}
  for i,v in ipairs(self) do
    table.insert(t, v)
  end
  return t
end

function VariableVector:__tostring()
  local t = {}
  for i,v in ipairs(self) do
    table.insert(t, tostring(v))
  end
  table.insert(t, string.format('[%s of size %d]', torch.type(self), self:size()))
  return table.concat(t, '\n')
end
