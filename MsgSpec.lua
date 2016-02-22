


local md5 = require 'md5'
local torch = require 'torch'
local ros = require 'utils'

local MsgSpec = torch.class('ros.MsgSpec', ros)

local msgspec_cache = {}

BUILTIN_TYPES = {
  "int8", "uint8", "int16", "uint16", "int32", "uint32",
  "int64", "uint64", "float32", "float64", "string", "bool",
  "byte", "char" }
for _,v in ipairs(BUILTIN_TYPES) do
   BUILTIN_TYPES[v] = v
end

EXTENDED_TYPES = { time={"uint32", "uint32"}, duration={"int32", "int32"} }

DEFAULT_PACKAGE = "std_msgs"

-- Check if given type is a built-in type.
-- @param type type to check
-- @return true if type is a built-in type, false otherwise
function ros.is_builtin_type(type)
  local t = base_type(type)
  return BUILTIN_TYPES[t] ~= nil or  EXTENDED_TYPES[t] ~= nil
end

-- Resolve the given type.
-- @param type to resolve
-- @param package to which the type should be resolve relatively
-- @return the given value if it is either a base type or contains a slash,
-- otherwise returns package/type.
local function resolve_type(type, package)
  if ros.is_builtin_type(type) or type:find("/") then
    return type
  else
    return string.format("%s/%s", package, type)
  end
end

-- Get message specification.
-- @param msg_type message type (e.g. std_msgs/String). The name must include
-- the package.
function ros.get_msgspec(msg_type, specstr)
  roslua.utils.assert_rospack()

  if not msgspec_cache[msg_type] then
    msgspec_cache[msg_type] = MsgSpec.new{type=msg_type, specstr=specstr}
  end

  return msgspec_cache[msg_type]
end

function MsgSpec:__init(o)
  assert(o.type, "Message type is missing")

  local slashpos = o.type:find("/")
  if slashpos then
    o.package    = o.type:sub(1, slashpos - 1)
    o.short_type = o.type:sub(slashpos + 1)
  else
    o.package    = DEFAULT_PACKAGE
    o.short_type = o.type
  end

   if o.specstr then
      o:load_from_string(o.specstr)
   else
      o:load()
   end

   o.base_format, o.base_farray = o:generate_base_format()
end

--- Load message specification from file.
-- Will search for the appropriate message specification file (using rospack)
-- and will then read and parse the file.
function MsgSpec:load()
   local package_path = roslua.utils.find_rospack(self.package)
   self.file = package_path .. "/msg/" .. self.short_type .. ".msg"

   return self:load_from_iterator(io.lines(self.file))
end


--- Load specification from string.
-- @param s string containing the message specification
function MsgSpec:load_from_string(s)
   return self:load_from_iterator(s:gmatch("(.-)\n"))
end

function MsgSpec:resolve_type(type)
   return resolve_type(type, self.package)
end


-- (internal) create string representation appropriate to generate the hash
-- @return string representation
function MsgSpec:generate_hashtext()
   local s = ""
   for _, spec in ipairs(self.constants) do
      s = s .. string.format("%s %s=%s\n", spec[1], spec[2], spec[3])
   end

   for _, spec in ipairs(self.fields) do
      if is_builtin_type(spec[1]) then
	 s = s .. string.format("%s %s\n", spec[1], spec[2])
      else
	 local msgspec = get_msgspec(base_type(spec[1]))
	 s = s .. msgspec:md5() .. " " .. spec[2] .. "\n"
      end
   end
   s = string.gsub(s, "^(.+)\n$", "%1") -- strip trailing newline

   return s
end

-- (internal) Calculate MD5 sum.
-- Generates the MD5 sum for this message type.
-- @return MD5 sum as text
function MsgSpec:calc_md5()
   self.md5sum = md5.sumhexa(self:generate_hashtext())
   return self.md5sum
end

function MsgSpec:md5()
 return self.md5sum or self:calc_md5()
end

function MsgSpec:print(indent)
  local indent = indent or ""
  print(indent .. "Message " .. self.type)
  print(indent .. "Fields:")
  for _,s in ipairs(self.fields) do
    print(indent .. "  " .. s[1] .. " " .. s[2])
    if not is_builtin_type(s[1]) then
      local msgspec = get_msgspec(base_type(s[1]))
      msgspec:print(indent .. "    ")
    end
  end

  print(indent .. "MD5:    " .. self:md5())
  print(indent .. "Format: " .. self.base_format)
end

function MsgSpec:__tostring()
  return self:print()
end
