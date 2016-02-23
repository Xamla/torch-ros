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

function Message:__init()
end
