local ros = require 'ros'

ros.init('service_client_demo')

local nh = ros.NodeHandle()


-- 1. Call a simple service with empty request message

local clientA = nh:serviceClient('/rosout/get_loggers', 'roscpp/GetLoggers')

-- we can check if the service exists
local ok = clientA:exists()
print('exists() returned: ' .. tostring(ok))

-- or wait for it to become available
local timeout = ros.Duration(5)
local ok = clientA:waitForExistence(timeout)
print('waitForExistence() returned: ' .. tostring(ok))


print('Calling service: ' .. clientA:getService())

-- call the service
local response = clientA:call()

print('Response:')
print(response)


-- 2. Call a service with a non-empty request.

local clientB = ros.ServiceClient('/rosout/set_logger_level', 'roscpp/SetLoggerLevel')

print('Service spec:')
print(clientB.spec)

-- we can either create the request message explicitely
local req_msg = clientB:createRequest()
req_msg:fillFromTable({logger="my_dummy_logger", level="warn"})

print('Request:')
print(req_msg)

print('Calling service: ' .. clientB:getService())

-- call the service
response = clientB:call(req_msg)

print('Response:')
print(response)

-- or let `call()` internally call fillFromTable() for us...
response = clientB:call{logger="my_dummy_logger", level="warn"}


ros.shutdown()
