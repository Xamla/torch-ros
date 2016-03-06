ros = require 'ros'

local sw = ros.StorageWriter()
sw:writeString('Hello, this is a string!')
for i=1,100 do 
  sw:writeInt16(i)
end
sw:writeFloat32(1.23)
sw:writeFloat64(1.23)
sw:writeTensor(torch.linspace(0,10,10))

local rw = ros.StorageReader(sw.storage)
print(rw:readString())
for i=1,100 do
  print(rw:readInt16())
end

print(rw:readFloat32())
print(rw:readFloat64())

print(rw:readDoubleTensor())

--[[
local header_spec = ros.MsgSpec('sensor_msgs/Image')

local msg = ros.Message(header_spec)
print(msg.spec)
print(msg)
]]