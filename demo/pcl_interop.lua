ros = require 'ros'
pcl = require 'pcl'
require 'ros.PointCloud2SerializationHandler'

ros.init('pcl_interop_demo')
nh = ros.NodeHandle()

local handler = ros.PointCloud2SerializationHandler()
nh:addSerializationHandler(handler)


publisher = nh:advertise('point_cloud_output', 'sensor_msgs/PointCloud2', 10)

print('press ctrl+c to exit')
while ros.ok() do
  if publisher:getNumSubscribers() > 0 then
    print('pub')
    local c = pcl.rand(100)   -- create dummy point cloud
    publisher:publish(c)
  end
  sys.sleep(0.5)
  ros.spinOnce()
end

nh:shutdown()
ros.shutdown()
