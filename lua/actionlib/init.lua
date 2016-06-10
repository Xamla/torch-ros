--- ROS actionlib
-- @module ros.actionlib
--
-- General documentation about the actionlib
-- http://wiki.ros.org/actionlib
-- http://wiki.ros.org/actionlib/DetailedDescription
--
-- C++ & Python source code:
-- https://github.com/ros/actionlib/tree/indigo-devel/include/actionlib

local ros = require 'ros.env'
require 'ros.ros'
require 'ros.Time'
require 'ros.Duration'
require 'ros.console'
require 'ros.StorageWriter'
require 'ros.StorageReader'
require 'ros.MsgSpec'
require 'ros.Message'
require 'ros.NodeHandle'
require 'ros.actionlib.ActionSpec'
local actionlib = ros.actionlib


return actionlib
