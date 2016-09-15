local ffi = require 'ffi'
local torch = require 'torch'
local ros = require 'ros.env'

local StorageWriter = torch.class('ros.StorageWriter', ros)
local SIZE_OF_UINT32 = ffi.sizeof('uint32_t')

local function ensurePosWriteable(self, pos, growth_factor)
  growth_factor = growth_factor or 2
  if self.capacity < pos then
    self:setCapacity(math.max(self.capacity * growth_factor, pos))
  end
  self.length = math.max(self.length, pos)
end

function StorageWriter:__init(storage, offset, serialization_handlers)  -- offset is zero based
  if not ffi.abi('le') then
    error('Big-endian systems not yet supported.')
  end

  self.offset = offset or 0
  self.serialization_handlers = serialization_handlers
  self.length = self.offset
  if not storage then
    self.storage = torch.ByteStorage()
    self:setCapacity(16)
  else
    if not torch.isTypeOf(storage, torch.ByteStorage) then
      error('argument 1: torch.ByteStorage expected')
    end
    self.storage = storage
    self.capacity = storage:size()
    self.data = self.storage:data()
  end
end

function StorageWriter:data()
  return self.storage:data()
end

function StorageWriter:getCapacity()
  return self.capacity
end

function StorageWriter:setCapacity(capacity)
  self.storage:resize(capacity)
  self.data = self.storage:data()
  self.capacity = capacity
end

function StorageWriter:storageChanged(newOffset)
  self.capacity = self.storage:size()
  self.offset = newOffset
  self.length = math.max(self.length, newOffset)
  self.data = self.storage:data()
end

function StorageWriter:shrinkToFit()
  self:setCapacity(self.length)
end

function StorageWriter:setLength(length, shrinkToFit)
  if not length or length < 0 then
    error('argument 1: positive length expected')
  end
  ensurePosWriteable(self, length, shrinkToFit and 0 or 1.5)
end

local function createWriteMethod(type)
  local element_size = ffi.sizeof(type)
  if ffi.arch == 'arm' then
    -- use ffi.copy() instead of plain cast on ARM to avoid bus errors
    local buffer = ffi.typeof(type .. '[1]')()
    return function(self, value, offset)
      local offset_ = offset or self.offset
      ensurePosWriteable(self, offset_ + element_size)
      buffer[0] = value
      ffi.copy(self.data + offset_, buffer, element_size)
      if not offset then
        self.offset = self.offset + element_size
      end
    end
  else
    local ptr_type = ffi.typeof(type .. '*')
    local element_size = ffi.sizeof(type)
    return function(self, value, offset)
      local offset_ = offset or self.offset
      ensurePosWriteable(self, offset_ + element_size)
      ffi.cast(ptr_type, self.data + offset_)[0] = value
      if not offset then
        self.offset = self.offset + element_size
      end
    end
  end
end

StorageWriter.writeInt8    = createWriteMethod('int8_t')
StorageWriter.writeInt16   = createWriteMethod('int16_t')
StorageWriter.writeInt32   = createWriteMethod('int32_t')
StorageWriter.writeInt64   = createWriteMethod('int64_t')
StorageWriter.writeUInt8   = createWriteMethod('uint8_t')
StorageWriter.writeUInt16  = createWriteMethod('uint16_t')
StorageWriter.writeUInt32  = createWriteMethod('uint32_t')
StorageWriter.writeUInt64  = createWriteMethod('uint64_t')
StorageWriter.writeFloat32 = createWriteMethod('float')
StorageWriter.writeFloat64 = createWriteMethod('double')

function StorageWriter:writeString(value)
  if type(value) ~= 'string' then
    error('argument 1: string expected')
  end
  ensurePosWriteable(self, self.offset + SIZE_OF_UINT32 + #value)
  self:writeUInt32(#value)                          -- write length of string
  ffi.copy(self.data + self.offset, value, #value)  -- copy string value
  self.offset = self.offset + #value
end

function StorageWriter:writeTensor(value, fixed_array_size)
  -- only tensors with a single dimension are supported for now (sufficient for ROS array support)
  if not torch.isTensor(value) or value:nDimension() > 1 then
    error('argument 1: tensor with one dimension expected')
  end

  value = value:contiguous()    -- ensure we are dealing with a contiguous piece of memory

  local n = value:nElement()
  local sizeInBytes = n * value:elementSize()

  if fixed_array_size == nil then
    ensurePosWriteable(self, self.offset + SIZE_OF_UINT32 + sizeInBytes)
    self:writeUInt32(n)                                           -- length of array
  else
    if n ~= fixed_array_size then
      error(string.format('Wrong number of elements in fixed size array (expected: %d; actual: %d).', fixed_array_size, n))
    end
    ensurePosWriteable(self, self.offset + sizeInBytes)
  end
  ffi.copy(self.data + self.offset, value:data(), sizeInBytes)  -- binary data
  self.offset = self.offset + sizeInBytes
end

local function createTypedWriteTensorMethod(tensor_ctor)
  return function(self, value)
    if not torch.isTypeOf(value, tensor_ctor) then
      error('argument 1: tensor has unexpeted type')
    end
    self:writeTensor(value)
  end
end

StorageWriter.writeByteTensor   = createTypedWriteTensorMethod(torch.ByteTensor)
StorageWriter.writeShortTensor  = createTypedWriteTensorMethod(torch.ShortTensor)
StorageWriter.writeIntTensor    = createTypedWriteTensorMethod(torch.IntTensor)
StorageWriter.writeLongTensor   = createTypedWriteTensorMethod(torch.LongTensor)
StorageWriter.writeFloatTensor  = createTypedWriteTensorMethod(torch.FloatTensor)
StorageWriter.writeDoubleTensor = createTypedWriteTensorMethod(torch.DoubleTensor)

function StorageWriter:getHandler(message_type)
  return self.serialization_handlers and self.serialization_handlers[message_type]
end
