local ffi = require 'ffi'
local torch = require 'torch'
local ros = require 'ros.env'

local Message = torch.class('ros.Message', ros)

-- (internal) table of default values for built-in types
local DEFAULT_VALUES = {
  bool     = 0,
  int8     = 0,       uint8   = 0,
  int16    = 0,       uint16  = 0,
  int32    = 0,       uint32  = 0,
  int64    = 0,       uint64  = 0,
  float32  = 0,       float64 = 0,
  char     = 0,       byte    = 0,
  duration = ros.Duration(),  time = ros.Time(),
  string   = ""
}

function Message:__init(spec, no_prefill)
  if not torch.isTypeOf(spec, ros.MsgSpec) and type(spec) == 'string' then
    spec = ros.MsgSpec(spec)
  end
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
        local v = DEFAULT_VALUES[f.type]
        if f.cloneable then
          v = v:clone()                                     -- clone time or duration
        end
        self.values[f.name] = v                             -- set default builtin value
      else
        self.values[f.name] = ros.Message.new(f.spec)       -- generate sub-message with default values
      end
    end
  end
end

--- Assign values from other message or table.
function Message:assign(source)
  if source == self then return end   -- ignore assignment to itself
  if torch.isTypeOf(source, ros.Message) then
    self:copy(source)
  elseif type(source) == 'table' then
    self:fillFromTable(source)
  end
end

--- Fill message fields from a source table.
function Message:fillFromTable(t)
  local fields = self.spec.fields
  for k,v in pairs(t) do
    local f = fields[k]   -- check if field or index exists
    if f ~= nil then
      if f.is_builtin then
        if f.type == 'duration' or f.type == 'time' then
          self.values[f.name]:set(v)
        else
          self.values[f.name] = v
        end
      else
        self.values[f.name]:assign(v)
      end
    end
  end
end

function Message:clone()
  local m = ros.Message(self.spec, true)
  for _,f in ipairs(self.spec.fields) do
    local v = self.values[f.name]
    if f.is_array then
      if f.tensor_type then
        v = v:clone()
      else
        local a = {}
        if f.cloneable then
           -- clone element-wise
          for i,x in ipairs(v) do
            a[i] = x:clone()
          end
        else
          -- copy array of immutable objects
          for i,x in ipairs(v) do
            a[i] = x
          end
        end
        v = a
      end
    else  -- single element field
      if f.cloneable then
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
          if f.cloneable then
            -- complex fields: copy element-wise
            for i,x in ipairs(v) do
              if a[i] then
                if f.is_builtin then
                  y[i]:set(x)       -- special handling for duration and time
                else
                  y[i]:copy(x)      -- use copy to not impact exsting references to target
                end
              else
                y[i] = x:clone()
              end
            end
          else
            -- copy array of immutable objects (simple builtin types)
            for i,x in ipairs(v) do
              y[i] = x
            end
          end

          -- remove additional array that are not present in v (the source array had less elements)
          while #y > #v do
            table.remove(y, #v)
          end
        end
      else    -- non array case: single element field
        if f.cloneable then
          self.values[f.name] = source.values[f.name]:clone()
        else
          self.values[f.name] = source.values[f.name]
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
  if fspec and fspec.is_array then

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
    sw:writeUInt32(value:get_sec())
    sw:writeUInt32(value:get_nsec())
  end,
  duration = function(sw, value)
    sw:writeInt32(value:get_sec())
    sw:writeInt32(value:get_nsec())
  end
}

local function serialize_inner(sw, self)

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
        local write
        if f.is_builtin then
          write = writeMethods[f.base_type]
          if not write then
            error(string.format('Mising write function for type \'%s\'.', f.base_type))
          end
        else
          write = serialize_inner
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
      serialize_inner(sw, v)
    end
  end
end

function Message:serialize(sw)
  sw = sw or ros.StorageWriter()
  local offset = sw.offset
  sw:writeUInt32(0)   -- reserve space for message size

  serialize_inner(sw, self)

  sw:writeUInt32(sw.offset - offset - 4, offset)

  return sw
end

function Message:serializeServiceResponse(sw, ok)
  sw = sw or ros.StorageWriter()

  local offset = sw.offset
  sw:writeUInt8(0)    -- reserve space for ok-flag

  if ok then
    sw:writeUInt32(0)   -- reserve space for message size

    serialize_inner(sw, self)

    sw:writeUInt8(1, offset)     -- ok-flag
    sw:writeUInt32(sw.offset - offset - 5, offset+1)
  else
    serialize_inner(sw, self)
    sw:writeUInt8(0, offset)
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
    return ros.Time(sr:readUInt32(), sr:readUInt32())
  end,
  duration = function(sr)
    return ros.Duration(sr:readInt32(), sr:readInt32())
  end
}

local function deserialize_internal(self, sr)
  for _, f in ipairs(self.spec.fields) do
    if f.is_array then
      if f.tensor_type then
        -- tensor
        if not self.values[f.name] then
          self.values[f.name] = f.tensor_type()
        end
        self.values[f.name]:set(sr:readTensor(f.tensor_type))
      else
        -- regular array
        local n = sr:readUInt32()    -- element count
        local read = readMethods[f.base_type]
        if not read then
          error(string.format('Mising read function for type \'%s\'.', f.base_type))
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
      local inner = self.values[f.name]
      if inner == nil then
        inner = ros.Message.new(f.spec, true)
        self.values[f.name] = inner
      end
      -- complex message
      deserialize_internal(inner, sr)
    end
  end
end

function Message:deserialize(sr)
  if torch.isTypeOf(sr, torch.ByteStorage) then
    sr = ros.StorageReader(sr)
  end

  if not sr then
    error('argument 1: storage reader object expected')
  end

  local totalLength = sr:readUInt32()
  deserialize_internal(self, sr)
  return sr
end
