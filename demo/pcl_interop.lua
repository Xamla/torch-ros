ros = require 'ros'
pcl = require 'pcl'
tf = ros.tf
require 'ros.PointCloud2SerializationHandler'

ros.init('pcl_interop_demo')
nh = ros.NodeHandle()

local handler = ros.PointCloud2SerializationHandler()
nh:addSerializationHandler(handler)

function onMessage(msg, header)
  print('received point cloud:')
  print(msg:toPointCloud():points())
end

publisher = nh:advertise('point_cloud_output', 'sensor_msgs/PointCloud2', 10)

-- establish intraprocess subscription
subscriber = nh:subscribe('point_cloud_output', 'sensor_msgs/PointCloud2', 10)
subscriber:registerCallback(onMessage)

print('press ctrl+c to exit')
while ros.ok() do
  if publisher:getNumSubscribers() > 0 then
    local c = pcl.rand(10)   -- create dummy point cloud
    c:setHeaderFrameId('/map')
    print('publishing point cloud:')
    print(c:points())
    publisher:publish(c)
  end
  sys.sleep(0.5)
  ros.spinOnce()
end

nh:shutdown()
ros.shutdown()
