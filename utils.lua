local ffi = require 'ffi'
local ros = require 'ros.env'

local utils = {}

function utils.create_method_table(prefix, names)
  local map = {}
  for i,n in ipairs(names) do
    local full_name = prefix .. n
    -- use pcall since not all types support all functions
    local ok,v = pcall(function() return moveit.lib[full_name] end)
    if ok then
      map[n] = v
    end
  end
  
  -- check whether we have new and delete functions
  -- automatically register objects created by new with the gc 
  local _new, _clone, _delete = map.new, map.clone, map.delete
  
  if _new and _delete then
    map.new = function(...)
      local obj = _new(...)
      ffi.gc(obj, _delete)
      return obj
    end
  end
  
  if _clone and _delete then
    map.clone = function(...)
      local obj = _clone(...)
      ffi.gc(obj, _delete)
      return obj
    end
  end
  
  return map
end

-- safe accessor for cdata()
function utils.cdata(x)
  return x and x:cdata() or moveit.NULL
end

return utils
