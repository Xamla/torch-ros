ros = require 'ros'


function base_test()
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
end


local img_spec = ros.MsgSpec('sensor_msgs/Image')

local msg = ros.Message(img_spec)
msg.header.seq = 918273
msg.width = 123
msg.height = 456
msg.step = 987
msg.data = torch.ByteTensor(10):fill(1)

v = msg:serialize()
v:shrinkToFit()

local msg2 = ros.Message(img_spec, true)
msg2:deserialize(v.storage)

print(msg2.spec)
print(msg2)


function testFixedSizeArray()
  local m = ros.Message('geometry_msgs/PoseWithCovariance')
  m.covariance[5] = 123
  local v = m:serialize()
  v:shrinkToFit()
  n = ros.Message('geometry_msgs/PoseWithCovariance')
  n:deserialize(v.storage)
  assert(n.covariance[5] == 123)
end

testFixedSizeArray()