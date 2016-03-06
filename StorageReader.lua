local ffi = require 'ffi'
local torch = require 'torch'
local ros = require 'ros.env'

local StorageReader = torch.class('ros.StorageReader', ros)
local SIZE_OF_UINT32 = ffi.sizeof('uint32_t')

local function ensurePosReadable(self, pos)
  if pos < 0 or pos >= self.length then
    error('Read position out of range.')
  end
end

function StorageReader:__init(storage, offset, length)
  if not ffi.abi('le') then
    error('Big-endian systems not yet supported.')
  end
  
  if not torch.isTypeOf(storage, torch.ByteStorage) then
    error('argument 1: torch.ByteStorage expected')
  end
  
  offset = offset or 0
  if offset < 0 or offset > storage:size() then
    error('argument 2: offset outside storage bounds')
  end
  
  self.storage = storage
  self.data = storage:data()
  self.offset = offset or 0
  self.length = length or storage:size()
  self.length = math.min(self.length, storage:size())
end

local function createReadMethod(type)
  local ptr_type = ffi.typeof(type .. '*')
  local element_size = ffi.sizeof(type)
  return function(self, offset)
    local offset_ = offset or self.offset
    ensurePosReadable(self, offset_ + element_size - 1)
    if not offset then
      self.offset = self.offset + element_size
    end
    return ffi.cast(ptr_type, self.data + offset_)[0]
  end
end

StorageReader.readInt8    = createReadMethod('int8_t')
StorageReader.readInt16   = createReadMethod('int16_t')
StorageReader.readInt32   = createReadMethod('int32_t')
StorageReader.readInt64   = createReadMethod('int64_t')
StorageReader.readUInt8   = createReadMethod('uint8_t')
StorageReader.readUInt16  = createReadMethod('uint16_t')
StorageReader.readUInt32  = createReadMethod('uint32_t')
StorageReader.readUInt64  = createReadMethod('uint64_t')
StorageReader.readFloat32 = createReadMethod('float')
StorageReader.readFloat64 = createReadMethod('double')

function StorageReader:readString(offset)
  local offset_ = offset or self.offset
  local length = self:readUInt32(offset)
  offset_ = offset_ + SIZE_OF_UINT32
  ensurePosReadable(self, offset_ + length - 1)
  if not offset then
    self.offset = offset_ + length
  end
  return ffi.string(self.data + offset_, length)
end

local function createReadTensorMethod(tensor_ctor)
  return function(self, offset)
    local offset_ = offset or self.offset
    local n = self:readUInt32(offset)
    local t = tensor_ctor()
    local sizeInBytes = n * t:elementSize()
    offset_ = offset_ + SIZE_OF_UINT32
    ensurePosReadable(self, offset_ + sizeInBytes - 1)
    t:resize(n)
    ffi.copy(t:data(), self.data + offset_, sizeInBytes)
    return t
  end
end

StorageReader.readByteTensor   = createReadTensorMethod(torch.ByteTensor)
StorageReader.readIntTensor    = createReadTensorMethod(torch.IntTensor)
StorageReader.readShortTensor    = createReadTensorMethod(torch.ShortTensor)
StorageReader.readLongTensor   = createReadTensorMethod(torch.LongTensor)
StorageReader.readFloatTensor  = createReadTensorMethod(torch.FloatTensor)
StorageReader.readDoubleTensor = createReadTensorMethod(torch.DoubleTensor)
