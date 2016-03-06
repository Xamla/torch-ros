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
          self.values[f.name] = f.tensor_type()             -- empty tensor
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

local writeMethods = {
  bool    = ros.StorageWriter.writeInt8,
  byte    = ros.StorageWriter.writeUInt8,
  char    = ros.StorageWriter.writeInt8,
  int8    = ros.StorageWriter.writeInt8,
  uint8   = ros.StorageWriter.writeUInt8, 
  int16   = ros.StorageWriter.writeInt16,
  uint16  = ros.StorageWriter.writeUInt16,
  int32   = ros.StorageWriter.writeInt32,
  uint32  = ros.StorageWriter.writeUInt32,
  int64   = ros.StorageWriter.writeInt64,
  uint64  = ros.StorageWriter.writeUInt64,
  float32 = ros.StorageWriter.writeFloat32,
  float64 = ros.StorageWriter.writeFloat64,
  string  = ros.StorageWriter.writeString,
  time    = function(sw, value)
    sw:writeUInt32(value[1])
    sw:writeUInt32(value[2])
  end,
  duration = function(sw, value)
    sw:writeInt32(value[1])
    sw:writeInt32(value[2])
  end
}

function Message:serialize(sw)
  sw = sw or ros.StorageWriter()

  for _, f in ipairs(self.spec.fields) do
    local v = self.values[f.name]
    if f.is_array then
      if f.tensor_type then
        -- tensor
        sw:writeTensor(v)
      else
        -- regular array
        local n = #v
        sw:writeUInt32(n)    -- element count
        local write = writeMethods[f.base_type]
        if not write then
          error(string.format('Mising write function for type \'%s\'.', f.base_type))
        end
        for i=1,n do
          write(sw, v[i])
        end
      end
    elseif f.is_builtin then
      -- builtin type
      local write = writeMethods[f.type]
      if not write then
        error(string.format('Mising write function for type \'%s\'.', f.type))
      end
      write(sw, v)
    else
      -- complex message
      v:serialize(sw)
    end
  end
  
  return sw
end

local readMethods = {
  bool    = ros.StorageReader.readInt8,
  byte    = ros.StorageReader.readUInt8,
  char    = ros.StorageReader.readInt8,
  int8    = ros.StorageReader.readInt8,
  uint8   = ros.StorageReader.readUInt8, 
  int16   = ros.StorageReader.readInt16,
  uint16  = ros.StorageReader.readUInt16,
  int32   = ros.StorageReader.readInt32,
  uint32  = ros.StorageReader.readUInt32,
  int64   = ros.StorageReader.readInt64,
  uint64  = ros.StorageReader.readUInt64,
  float32 = ros.StorageReader.readFloat32,
  float64 = ros.StorageReader.readFloat64,
  string  = ros.StorageReader.readString,
  time    = function(sr)
    return { sr:readUInt32(), sr:readUInt32() }
  end,
  duration = function(sr)
    return { sr:readInt32(), sr:readInt32() }
  end
}

function Message:deserialize(sr)
  if torch.isTypeOf(sr, torch.ByteStorage) then
    sr = ros.StorageReader(sr)
  end
  if not sr then
    error('argument 1: storage reader object expected')
  end

  for _, f in ipairs(self.spec.fields) do
    if f.is_array then
      if f.tensor_type then
        -- tensor
        self.values[f.name]:set(sr:readTensor(f.tensor_type))
      else
        -- regular array
        local n = sr:readUInt32(n)    -- element count
        local read = writeMethods[f.base_type]
        if not read then
          error(string.format('Mising write function for type \'%s\'.', f.base_type))
        end
        local t = {}
        for i=1,n do
          t[#t] = read(sr)
        end
        self.values[f.name] = t
      end
    elseif f.is_builtin then
      -- builtin type
      local read = readMethods[f.type]
      if not read then
        error(string.format('Mising read function for type \'%s\'.', f.type))
      end
      self.values[f.name] = read(sr)
    else
      -- complex message
      self.values[f.name]:deserialize(sr)
    end
  end
  
  return sr
end
