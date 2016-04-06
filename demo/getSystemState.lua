local ros = require 'ros'

ros.init('getSystemStateDemo')

-- execute the getSystemState state call via XMLRPC.
-- Master-API reference: http://wiki.ros.org/ROS/Master_API
local status,response,payload = ros.master.execute('getSystemState', ros.this_node.getName())

local function printList(list)
  for i,v in ipairs(list) do
    local topic = v[1]
    local nodes = v[2]
    print(string.format("Topic: '%s'", topic))
    print('  Nodes:')
    for j,n in ipairs(nodes) do
      print('    ' .. n)
    end
  end
end

if status then
  local publishers, subscribers, services = unpack(payload)
  print(string.format("# Publishers (%d)", #publishers))
  printList(publishers)
  print("")
  print(string.format("# Subscribers (%d)", #subscribers))
  printList(subscribers)
  print("")
  print(string.format("# Services (%d)", #services))
  printList(services)
  print("")
else
  print('getSystemState request failed.')
end

ros.shutdown()
