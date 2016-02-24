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
   bool     = "I1",
   int8     = "i1",    uint8   = "I1",
   int16    = "i2",    uint16  = "I2",
   int32    = "i4",    uint32  = "I4",
   int64    = "i8",    uint64  = "I8",
   float32  = "f",     float64 = "d",
   char     = "i1",    byte    = "I1",
   duration = "i4i4",  time    = "I4I4",
   string   = "i4c0",  array   = "I4"
}

function Message:__init(spec)
  assert(spec, "Message specification must not be nil.")
  self.spec = spec
  self.values = {}
end


function Message:deserialize(buffer)
  local farray = self.spec.base_farray
  for j, f in ipairs(farray) do
    if type(f) == "string" then
    
    else
    
    end
  end
end

-- write to an underlying byte stream
local function read_numeric(type)
  local ptr = ffi.type(type .. '*')()
  local size = ffi.sizeof(type)
  return function(p)
    return ffi.cast(p, ptr_ct)[0], p + size
  end
end

local read_table = 
{  
  i1 = read_numeric('int8_t'),
  i2 = read_numeric('int16_t'),
  i4 = read_numeric('int32_t'),
  i8 = read_numeric('int64_t'),
  I1 = read_numeric('uint8_t'),
  I2 = read_numeric('uint16_t'),
  I4 = read_numeric('uint32_t'),
  I8 = read_numeric('uint64_t'),
  f = read_numeric('float'),
  d = read_numeric('double')
  c0 = 
}


function interpret(ptr, format)


  for i=1,#format do
    format:byte(i)
  end
end

local types = {
  'bool*',
  'uint8_t*',
  'int8_t*',
  'int16_t*',
}

local pos = 1

function read(ct)
  ptr = ptr + sizeof()
end


