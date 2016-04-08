local md5 = require 'md5'
local path = require 'pl.path'
local torch = require 'torch'
local ros = require 'ros.env'

local SrvSpec = torch.class('ros.SrvSpec', ros)

local srvspec_cache = {}
local DEFAULT_PACKAGE = 'roslib'

--- Get service specification.
-- @param srv_type service type (e.g. roscpp/SetLoggerLevel). The name must include
-- the package.
local function get_srvspec(srv_type, specstr)
  if not srvspec_cache[srv_type] then
    srvspec_cache[srv_type] = ros.SrvSpec(srv_type, specstr)
  end

  return srvspec_cache[srv_type]
end
ros.get_srvspec = get_srvspec

--- (internal) load from iterator
-- @param iterator iterator that returns one line of the specification at a time
local function load_from_iterator(self, iterator)
  local request, response  = {}, {}

  -- extract the request and response message descriptions
  local t = request
  for line in iterator do
    if string.find(line, '^%s*---%s*$') ~= nil then
      t = response
    else
      table.insert(t, line)
    end
  end

  self.request_spec  = ros.MsgSpec(self.type .. '_Request', table.concat(request, '\n'))
  self.response_spec = ros.MsgSpec(self.type .. '_Response', table.concat(response, '\n'))
end

local function load_srvspec(self)
  local package_path = ros.find_package(self.package)
  self.file = path.join(package_path, 'srv', self.short_type .. ".srv")
  return load_from_iterator(self, io.lines(self.file))
end

--- (internal) Load specification from string.
-- @param s string containing the message specification
local function load_from_string(self, s)
  return load_from_iterator(self, s:gmatch('([^\r\n]+)\n?'))
end

-- (internal) Calculate MD5 sum.
-- Generates the MD5 sum for this message type.
-- @return MD5 sum as text
local function calc_md5(self)
  local s = self.request_spec:generate_hashtext() .. self.response_spec:generate_hashtext()
  self.md5sum = md5.sumhexa(s)
  return self.md5sum
end

function SrvSpec:__init(type, specstr)
  assert(type, 'Service type is expected')
  self.type = type

  local slashpos = type:find('/')
  if slashpos then
    self.package    = type:sub(1, slashpos - 1)
    self.short_type = type:sub(slashpos + 1)
  else
    self.package    = DEFAULT_PACKAGE
    self.short_type = type
  end

  if specstr then
    load_from_string(self, specstr)
  else
    load_srvspec(self)
  end
end

--- Get MD5 sum of type specification.
-- This will create a text representation of the service specification and
-- generate the MD5 sum for it. The value is cached so concurrent calls will
-- cause the cached value to be returned
-- @return MD5 sum of message specification
function SrvSpec:md5()
  return self.md5sum or calc_md5(self)
end

function SrvSpec:format_spec(ln)
  table.insert(ln, 'Service ' .. self.type)
  table.insert(ln, 'MD5: ' .. self:md5())
  self.request_spec:format_spec(ln)
  table.insert(ln, '---')
  self.response_spec:format_spec(ln)
  return ln
end

function SrvSpec:__tostring()
  lines = self:format_spec({})
  table.insert(lines, '')
  return table.concat(lines, '\n')
end
