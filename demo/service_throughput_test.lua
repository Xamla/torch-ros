local torch = require 'torch'
local ros = require 'ros'


local capture_spec = ros.SrvSpec('ximea_msgs/Capture', [[
string[] serials                      # serial numbers of cameras to use (empty means all cameras)
---
string[] serials                      # serial numbers of cameras
sensor_msgs/Image[] images            # image data
]])


local function server()
  ros.init('service_throughput_test_server')

  local nh = ros.NodeHandle()

  local imageMsg = ros.Message('sensor_msgs/Image')

  local function captureHandler(request, response, header)
    ros.INFO('incoming request')
    imageMsg.data = torch.ByteTensor(4096 * 4096)
    response.images[1] = imageMsg
    return true
  end

  nh:advertiseService('/image_source', capture_spec, captureHandler)
  ros.INFO('service registered')

  while ros.ok() do
    ros.spinOnce()
  end

  ros.shutdown()
end


local function client()
  ros.init('service_throughput_test_client')
  local nh = ros.NodeHandle()

  local svc_client = nh:serviceClient('/image_source', capture_spec, true)
  print('waiting for service to become available...')
  svc_client:waitForExistence()
  svc_client = nh:serviceClient('/image_source', capture_spec, true)

  local start_time = torch.tic()
  local data_received = 0
  for i=1,100 do
    ros.INFO('Calling: %d', i)
    local response = svc_client:call()
    data_received = data_received + response.images[1].data:storage():size()
    ros.INFO('Response received')
  end
  local elapsed = torch.toc(start_time)
  ros.INFO('Test took: %f', elapsed)
  ros.INFO('%f MB/s', data_received / elapsed / (1024 * 1024))

  ros.shutdown()
end


local function main()
  local mode = arg[1]

  if mode == 'client' then
    client()
  elseif mode == 'server' then
    server()
  else
    print('Please specify \'client\' or \'server\' as command line argument.')
  end

end


main()
