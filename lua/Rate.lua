--- ROS data class to handle time intervals
-- @classmod Rate

local ffi = require 'ffi'
local torch = require 'torch'
local ros = require 'ros.env'
local utils = require 'ros.utils'

local Rate = torch.class('ros.Rate', ros)

function init()
    local Rate_method_names = {
        'new',
        'clone',
        'delete',
        'reset',
        'sleep',
        'expectedCycleTime',
        'cycleTime'
    }

    return utils.create_method_table('ros_Rate_', Rate_method_names)
end

local f = init()

--- Construct a time rate
-- @tparam double _1 frequency 	The desired rate to run at in Hz
function Rate:__init(_1)
    self.o = f.new(_1)
end

--- Get the underlying data structure
-- @return
function Rate:cdata()
    return self.o
end

--- Creates a deep copy of the object
-- @return A copy of the object
function Rate:clone()
    local c = torch.factory('ros.Rate')()
    rawset(c, 'o', f.clone(self.o))
    return c
end

---Sets the start time for the rate to now.
function Rate:reset()
    f.reset(self.o)
end

--- Sleeps for any leftover time in a cycle. Calculated from the last time sleep, reset, or the constructor was called.
function Rate:sleep()
    f.sleep(self.o)
end

--- Get the expected cycle time -- one over the frequency passed in to the constructor.
function Rate:expectedCycleTime(output)
    local output = output or ros.Duration()
    f.expectedCycleTime(self.o, output:cdata())
    return output
end

--- Get the actual run time of a cycle from start to sleep.
function Rate:cycleTime(output)
    local output = output or ros.Duration()
    f.cycleTime(self.o, output:cdata())
    return output
end

function Rate:__tostring()
    return string.format(
        'cycleTime: %fsec, expectedCycleTime; %fsec',
        self:cycleTime():toSec(),
        self:expectedCycleTime():toSec()
    )
end
