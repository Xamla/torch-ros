local ffi = require 'ffi'
local torch = require 'torch'
local ros = require 'ros.env'

local Message = torch.class('ros.Message', ros)

-- (internal) table of default values for built-in types
Message.default_values = {
  bool     = 0,
  int8     = 0,       uint8   = 0,
  int16    = 0,       uint16  = 0,
  int32    = 0,       uint32  = 0,
  int64    = 0,       uint64  = 0,
  float32  = 0,       float64 = 0,
  char     = 0,       byte    = 0,
  duration = {0, 0},  time    = {0, 0},
  string   = "",      array   = {}
}

-- (internal) table of formats for built-in types
Message.builtin_formats = {
  bool     = "B",
  int8     = "c",    uint8   = "C",
  int16    = "w",    uint16  = "W",
  int32    = "i",    uint32  = "I",
  int64    = "q",    uint64  = "Q",
  float32  = "f",    float64 = "F",
  char     = "B",    byte    = "B",
  duration = "ii",   time    = "II",
  string   = "s",    array   = "I"
}

function Message:__init(spec, no_prefill)
  assert(spec, "Message specification must not be nil.")
  rawset(self, 'spec', spec)
  rawset(self, 'values', {})
  if not no_prefill then 
    self:prefill() 
  end
end

function Message:__index(key)
  local v = rawget(self, idx)
  if not v then
    v = Message[key]
    if not v then
      v = rawget(self, 'values')[key]
    end
  end
  return v
end

function Message:__newindex(key, value)
  self.values[key] = value
end

--- Prefill message with default values.
function Message:prefill()
  for _, f in ipairs(self.spec.fields) do
    if not self.values[f.name] then
      if f.is_array then
        if f.tensor_type then
          self.values[f.name] = f.tensor_type:new()         -- empty tensor
        else
          self.values[f.name] = {}                          -- empty array
        end
      elseif f.is_builtin then
        self.values[f.name] = self.default_values[f.type]   -- set default builtin value
      else 
        self.values[f.name] = ros.Message.new(f.spec)       -- generate sub-message with default values
      end
    end
  end
end

function Message:clone()
  local m = ros.Message:new(self.spec, true)
  for _,f in ipairs(self.spec.fields) do
    local v = self.values[f.name]
    if f.is_array then
      if f.tensor_type then
        v = v:clone()
      else
        local a = {}
        if f.is_builtin then
          if f.base_type == 'duration' or f.base_type == 'time' then
            -- special handling of duration and time fields
            for i,x in ipairs(v) do
              a[i] = { x[1], x[2] }
            end
          else
            -- copy array of immutable objects
            for i,x in ipairs(v) do
              a[i] = x
            end
          end
        else
          -- clone element-wise
          for i,x in ipairs(v) do
            a[i] = x:clone()
          end
        end
        v = a
      end
    else  -- single element field
      if not f.is_builtin then
        v = v:clone()
      end
    end
    m.values[f.name] = v
  end
  return m 
end

function Message:copy(source)
  for _,f in ipairs(self.spec.fields) do  -- for all target fields
    local v = source.values[f.name]   -- source value
    local y = self.values[f.name]     -- destination value
    if v ~= nil then  -- source value exists
      if f.is_array then
        if f.tensor_type then
          y:resizeAs(v)
          y:copy(v)
        else
          y = y or {}
          if f.is_builtin then
            if f.base_type == 'duration' or f.base_type == 'time' then
              -- special handling of duration and time fields
              for i,x in ipairs(v) do
                y[i] = { x[1], x[2] }
              end
            else
              -- copy array of immutable objects (builtin types not represented as table)
              for i,x in ipairs(v) do
                y[i] = x
              end
            end
          else
            -- complex fields: copy element-wise
            for i,x in ipairs(v) do
              if a[i] then
                y[i]:copy(x)      -- use copy to not impact exsting references to target
              else
                y[i] = x:clone()
              end
            end
          end
          
          -- remove additional array that are not present in v (the source array had less elements)
          while #y > #v do
            table.remove(y, #v)
          end
        end
      else    -- non array case: single element field
        if f.is_builtin then
          self.values[f.name] = source.values[f.name]
        else
          self.values[f.name] = source.values[f.name]:clone()
        end
      end
    end
  end
end

local format_field

local function format_message(msg, ln, indent)
  local indent = indent or ''
  table.insert(ln, indent .. msg.spec.type .. ' {')
  for _, fspec in ipairs(msg.spec.fields) do
    local fname = fspec.name
    local ftype = fspec.type    
    local fvalue = msg.values[fname]
    local prefix = indent .. '  ' .. fname .. ' : '
    format_field(ln, indent, prefix, ftype, fspec, fvalue)
  end
  table.insert(ln, indent .. '}')
  return ln
end

format_field = function(ln, indent, prefix, ftype, fspec, fvalue)
  if ftype == 'time' or ftype == 'duration' then
    table.insert(ln, prefix .. fvalue[1] .. '.' .. fvalue[2])
  elseif fspec and fspec.is_array then
    
    if fspec.tensor_type then
      if fvalue and fvalue:nElement() > 100 then
        local tensor_size = table.concat(fvalue:size():totable(), 'x')
        table.insert(ln, prefix .. '[' .. fvalue:type() .. ' of size ' .. tensor_size .. ']')
      else
        table.insert(ln, prefix)
        table.insert(ln, tostring(fvalue))
      end
    elseif #fvalue == 0 then
      table.insert(ln, prefix .. "[]")
    else
      local t = { prefix, '[\n' }
      
      if fspec.is_builtin then
        for i,x in ipairs(fvalue) do
          format_field(t, indent .. '  ', '', fspec.base_type, nil, x)
          table.insert(t, ',\n')
        end
      else
        for i,a in ipairs(fvalue) do
          format_message(a, t, indent .. '  ')
          table.insert(t, ',\n')
        end
      end
      table.insert(t, indent .. ']')
      table.insert(ln, table.concat(t))
    end
    
  elseif ftype == 'string' then
    table.insert(ln, prefix .. '"' .. fvalue .. '"')
  elseif fspec and not fspec.is_builtin then
    table.insert(ln, prefix)
    format_message(fvalue, ln, indent .. '  ')
  else
    table.insert(ln, prefix .. tostring(fvalue))
  end
end

function Message:__tostring()
  local lines = format_message(self, {})
  table.insert(lines, '')
  return table.concat(lines, '\n')
end

-- write to an underlying byte stream
local function create_read_numeric(type)
  local ptr_type = ffi.typeof(type .. '*')
  local size = ffi.sizeof(type)
  return function(p)
    return ffi.cast(ptr_type, p)[0], p + size
  end
end

local function create_read_string()
  local uint32_ptr = ffi.typeof('uint32_t*')
  return function(p)
    local length = ffi.cast(p, uint32_ptr)[0]
    p = p + 4
    return ffi.string(p, length), p + length
  end
end

local read_table = 
{  
  b = create_read_numeric('int8_t'),
  w = create_read_numeric('int16_t'),
  i = create_read_numeric('int32_t'),
  q = create_read_numeric('int64_t'),
  B = create_read_numeric('uint8_t'),
  W = create_read_numeric('uint16_t'),
  D = create_read_numeric('uint32_t'),
  Q = create_read_numeric('uint64_t'),
  f = create_read_numeric('float'),
  F = create_read_numeric('double'),
  s = create_read_string()
}

-- Add numeric conversions of characters to 
-- allow format interpretation without string
-- allocation.
for k,v in pairs(read_table) do
  if type(k) == 'string' then
    read_table[string.byte(k)] = v
  end
end

local function unpack_table(format, buffer, offset)
  local t = {}
  
  local p = buffer
  local c, fn, v
  
  for i=1,#format do
    c = format:byte(i)
    fn = read_table[c]
    if not fn then
      error(string.format('Mising read function for type \'%c\' at pos %d.', i, c))
    end
    v, ptr = fn(ptr)     -- decode value
    t[#t+1] = v
  end
  
  return t
end

function Message:sizeInBytes()
  
end

function Message:serialize()
  -- 
end

function Message:deserialize(buffer)
  local farray = self.spec.base_farray
  for j, f in ipairs(farray) do
    if type(f) == "string" then
    
    else
    
    end
  end
end

-- determine total message size

-- serialization method

-- create byte stream class to serialize to memory or file via ffi?

-- create write methods

local function write_field(strage, offset, ftype, fvalue)
  
end

function Message:write(storage, offset)

  for _, f in ipairs(self.spec.fields) do
    if f.is_array then
      if f.tensor_type then
        -- tensor
        
      else
        -- regular array
        
      end
    elseif f.is_builtin then
      -- builtin type
      offset = write_field(storage, offset, f.type, self.values[f.name])
    else
      -- complex message
      offset = f.values[f.name]:write(storage, offset)
    end
  end
end

