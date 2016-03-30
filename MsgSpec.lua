local md5 = require 'md5'
local path = require 'pl.path'
local torch = require 'torch'
local ros = require 'ros.env'

local MsgSpec = torch.class('ros.MsgSpec', ros)

local msgspec_cache = {}

local BUILTIN_TYPES = {
  'bool',
  'char', 'byte',
  'int8', 'uint8', 
  'int16', 'uint16', 
  'int32', 'uint32', 
  'int64', 'uint64', 
  'float32',
  'float64', 
  'string'
}

for _,v in ipairs(BUILTIN_TYPES) do
  BUILTIN_TYPES[v] = v
end

local EXTENDED_TYPES = {
  time = { 'uint32', 'uint32' },
  duration = { 'int32', 'int32' }
}

local DEFAULT_PACKAGE = 'std_msgs'

--- Check if type is an array type.
-- @param type type to check
-- @return true if given type is an array type, false otherwise
local function is_array_type(type)
  return type:find("%[") ~= nil
end

--- Get the base version of type, i.e. the non-array type.
-- @param type type to get base type for
-- @return base type, for array types returns the non-array type, for non-array
-- type returns the given value.
local function base_type(type)
   return type:match("^([^%[]+)") or type
end

--- Get message specification.
-- @param msg_type message type (e.g. std_msgs/String). The name must include
-- the package.
local function get_msgspec(msg_type, specstr)
  if not msgspec_cache[msg_type] then
    msgspec_cache[msg_type] = ros.MsgSpec(msg_type, specstr)
  end

  return msgspec_cache[msg_type]
end
ros.get_msgspec = get_msgspec

--- Check if given type is a built-in type.
-- @param type type to check
-- @return true if type is a built-in type, false otherwise
local function is_builtin_type(type)
  local t = base_type(type)
  return BUILTIN_TYPES[t] ~= nil or EXTENDED_TYPES[t] ~= nil
end
ros.is_builtin_type = is_builtin_type

local tensor_type_map = {
  byte = torch.ByteTensor,
  int8 = torch.CharTensor,
  uint8 = torch.ByteTensor,
  int16 = torch.ShortTensor,
  uint16 = torch.ShortTensor,
  int32 = torch.IntTensor,
  uint32 = torch.IntTensor,
  float32 = torch.FloatTensor,
  float64 = torch.DoubleTensor
}

--- (internal) load from iterator
-- @param iterator iterator that returns one line of the specification at a time
local function load_from_iterator(self, iterator)
  local lines = {}
  self.fields = {}
  self.constants = {}

  local field_index = 1
  local fixed_size = true

  for line in iterator do
    table.insert(lines, line)
    line = line:match('^([^#]*)') or ''     -- strip comment
    line = line:match('(.-)%s+$') or line   -- strip trailing whitespace

    if line ~= '' then
      local ftype, fname = string.match(line, '^([%w_/%[%]]+)[%s]+([%w_%[%]]+)$')
      if ftype and fname then
        if ftype == 'Header' then ftype = DEFAULT_PACKAGE .. '/Header' end
        ftype = self:resolve_type(ftype)
        
        local msgspec
        if not is_builtin_type(ftype) then
          msgspec = get_msgspec(base_type(ftype))   -- load sub-spec
          fixed_size = msgspec.fixed_size
        end
        
        local typeinfo = { 
          -- allow array like access by index
          ftype, fname, msgspec,

          -- and access by key
          type = ftype,
          name = fname,
          base_type = base_type(ftype),
          spec = msgspec,
          is_array = is_array_type(ftype),
          is_builtin = is_builtin_type(ftype),
          value_index = field_index
        }

        -- check tensor mapping
        if typeinfo.is_array then
          typeinfo.tensor_type = tensor_type_map[typeinfo.base_type]
          fixed_size = false
        elseif typeinfo.is_builtin and typeinfo.type == 'string' then
          fixed_size = false
        end

        self.fields[fname] = typeinfo
        table.insert(self.fields, typeinfo)
        field_index = field_index + 1
      else -- check for constant
        local ctype, cname, cvalue =
          line:match('^([%w_]+)[%s]+([%w_]+)[%s]*=[%s]*([%w%s\'._-]+)$')
        if ctype and cname and cvalue then
          local nv = tonumber(cvalue)
          if nv ~= nil then cvalue = nv end
          self.constants[cname] = { ctype, cvalue, type=ctype, value=cvalue }
          table.insert(self.constants, {ctype, cname, cvalue})
        else
          error(self.type .. ' invalid line: ' .. line)
        end
      end
    end

   end

   self.fixed_size = fixed_size
   self.definition = table.concat(lines, '\n')
end

--- (internal) Load message specification from file.
-- Will search for the appropriate message specification file (using rospack)
-- and will then read and parse the file.
local function load_msgspec(self)
  local package_path = ros.find_package(self.package)
  self.file = path.join(package_path, 'msg', self.short_type .. '.msg')
  return load_from_iterator(self, io.lines(self.file))
end

--- (internal) Load specification from string.
-- @param s string containing the message specification
local function load_from_string(self, s)
  return load_from_iterator(self, s:gmatch('(.-)$'))
end

--- (internal) Calculate MD5 sum.
-- Generates the MD5 sum for this message type.
-- @return MD5 sum as text
local function calc_md5(self)
  self.md5sum = md5.sumhexa(self:generate_hashtext())
  return self.md5sum
end

--- Resolve the given type.
-- @param type to resolve
-- @param package to which the type should be resolve relatively
-- @return the given value if it is either a base type or contains a slash,
-- otherwise returns package/type.
local function resolve_type(type, package)
  if is_builtin_type(type) or type:find('/') then
    return type
  else
    return string.format('%s/%s', package, type)
  end
end

function MsgSpec:__init(type, specstr)
  assert(type, 'Message type is expected')
  self.type = type
  
  local slashpos = type:find('/')
  if slashpos then
    self.package    = type:sub(1, slashpos - 1)
    self.short_type = type:sub(slashpos + 1)
  else
    self.package    = DEFAULT_PACKAGE
    self.short_type = type
  end

  if specstr then
    load_from_string(self, specstr)
  else
    load_msgspec(self)
  end
end

function MsgSpec:resolve_type(type)
  return resolve_type(type, self.package)
end

--- create string representation appropriate to generate the hash
-- @return string representation
function MsgSpec:generate_hashtext()
  local lines = {}
  for _, spec in ipairs(self.constants) do
    if #lines > 0 then table.insert(lines, '\n') end
    table.insert(lines, string.format('%s %s=%s', spec[1], spec[2], spec[3]))
  end

  for _, spec in ipairs(self.fields) do
    if #lines > 0 then table.insert(lines, '\n') end
    if is_builtin_type(spec[1]) then
      table.insert(lines, string.format('%s %s', spec[1], spec[2]))
    else
      local type_md5 = get_msgspec(base_type(spec[1])):md5()
      table.insert(lines, string.format('%s %s', type_md5, spec[2]))
    end
  end
  return table.concat(lines)
end

function MsgSpec:md5()
  return self.md5sum or calc_md5(self)
end

function MsgSpec:format_spec(ln, indent)
  local indent = indent or ''
  table.insert(ln, indent .. 'Message ' .. self.type)
  table.insert(ln, indent .. 'Fields:')
  for _,s in ipairs(self.fields) do
    table.insert(ln, indent .. '  ' .. s[1] .. ' ' .. s[2])
    if not is_builtin_type(s[1]) then
      local msgspec = get_msgspec(base_type(s[1]))
      msgspec:format_spec(ln, indent .. '    ')
    end
  end
  table.insert(ln, indent .. 'MD5:    ' .. self:md5())
  return ln
end

function MsgSpec:instantiate(no_prefill)
  return ros.Message:new(self, no_prefill)
end

function MsgSpec:__tostring()
  local lines = self:format_spec({})
  table.insert(lines, '')
  return table.concat(lines, '\n')
end
