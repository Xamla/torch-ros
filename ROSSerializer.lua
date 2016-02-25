local ffi = require 'ffi'
local torch = requrie 'torch'

-- parse message spec

--[[

Example point cloud message:

std_msgs/Header:
uint32 seq
time stamp
string frame_id

geometry_msgs/Point32:
float32 x
float32 y
float32 z

sensor_msgs/ChannelFloat32
string name 
float32[] values

sensor_msgs/PointCloud:
Header header
geometry_msgs/Point32[] points
ChannelFloat32[] channels

]]


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


function Message:deserialize(buffer)
   local format = self:format_string(buffer, 1, self.farray, "")

   local values = {}
   local l = 0
   local i = 1
   for s in self:split_format(format) do
      l = l + #s
      local tmpval = struct.unpack_table("<!1" .. s, buffer, i)
      i = tmpval[#tmpval]
      table.remove(tmpval)
      for _, v in ipairs(tmpval) do
	 table.insert(values, v)
      end
   end
   self:read_values(values)
end
