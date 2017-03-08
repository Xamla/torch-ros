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
msg.width = 5
msg.height = 6
msg.step = 5
msg.data = torch.range(2,60,2):byte() -- creates torch.ByteTensor with 30 elements

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
  
  local test_msg_definiton = [[Header header
uint32[5] id
string[4] names
time[2] times
float64 confidence
]]
  local s = ros.MsgSpec('test', test_msg_definiton)
  
  local now = ros.Time.now()
  m = ros.Message(s)
  m.id[1] = 1
  m.id[4] = 4
  m.names[2] = 'hallo'
  m.times[2] = now
  local v = m:serialize()
  v:shrinkToFit()
  x = ros.Message(s)
  x:deserialize(v.storage)
  assert(1 == x.id[1])
  assert(4 == x.id[4])
  assert('hallo' == x.names[2])
  assert(now == x.times[2])
  assert(x.times[1] == ros.Time(0))
end

testFixedSizeArray()
