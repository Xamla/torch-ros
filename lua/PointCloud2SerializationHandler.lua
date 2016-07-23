local ffi = require 'ffi'
local torch = require 'torch'
local ros = require 'ros.env'
local utils = require 'ros.utils'
local pcl = require 'pcl'
local std = ros.std

local PointCloud2SerializationHandler = torch.class('ros.PointCloud2SerializationHandler', ros)

function init()
  local names = {
    'readPointCloud2',
    'writePointCloud2'
  }

  return utils.create_method_table("ros_pcl_", names)
end

local f = init()

function PointCloud2SerializationHandler:init()
end

function PointCloud2SerializationHandler:getType()
  return "sensor_msgs/PointCloud2"
end

function PointCloud2SerializationHandler:read(sr, value)
  -- call deserialization function
  value = value or pcl.PCLPointCloud2()
  local newOffset = f.readPointCloud2(sr.storage:cdata(), sr.offset, value:cdata())
  sr:setOffset(newOffset)
  return value
end

function PointCloud2SerializationHandler:write(sw, value)
  if torch.isTypeOf(value, pcl.PointCloud) then
    value = value:toPCLPointCloud2()
  end

  if not torch.isTypeOf(value, pcl.PCLPointCloud2) then
    error("Invalid value type. 'pcl.PCLPointCloud2' expected.")
  end

  -- call serialization function
  local newOffset = f.writePointCloud2(sw.storage:cdata(), sw.offset, utils.cdata(value))
  sw:storageChanged(newOffset)
end
