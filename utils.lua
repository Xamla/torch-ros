local xlua = require 'xlua' -- string.split

local ros = {}

local ROS_VER
local rospack_path_cache = {}

-- Assert availability of rospack.
-- Throws an error if rospack cannot be executed, for example because ROS is
-- not installed or the binary is not in the PATH.
local asserted_rospack = false
function assert_rospack()
  if not asserted_rospack then
    local rv = os.execute("rospack 2>/dev/null")
    assert(rv == 0, "Cannot find rospack command, must be in PATH")
    asserted_rospack = true
 end
end

-- Get ROS version
function ros.version()
  if not ROS_VER then
    assert_rospack()
    local p = io.popen("rosversion roscpp 2>/dev/null")
    local version = p:read("*l")
    p:close()
    
    if not version or #version == 0 then
      error("Cannot determine ROS version")
    end
    
    local p = io.popen("rosversion -d 2>/dev/null")
    local codename = p:read("*l")
    p:close()
    
    local v = string.split(version, "%.")
    ROS_VER = {
      version = { tonumber(v[1]), tonumber(v[2]), tonumber(v[3]) },
      codename = codename
    }
 end
 return ROS_VER
end

-- Get path for a package
-- Uses rospack to find the path to a certain package. The path is cached so
-- that consecutive calls will not trigger another rospack execution, but are
-- rather handled directly from the cache. An error is thrown if the package
-- cannot be found.
-- @return path to give package
function ros.find_package(package)
  if not rospack_path_cache[package] then
    local p = io.popen("rospack find " .. package .. " 2>/dev/null")
    rospack_path_cache[package] = p:read("*l")
    p:close()
  end

  assert(rospack_path_cache[package] and #rospack_path_cache[package] > 0, 
    "Package path could not be found for " .. package)
  return rospack_path_cache[package]
end

return ros
