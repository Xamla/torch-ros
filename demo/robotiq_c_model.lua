ros = require 'ros'

--[[
make sure robitq gripper node is running, e.g. for connection via modbus RTU run:

rosrun robotiq_c_model_control CModelRtuNode.py /dev/ttyUSB1
]]

ros.init('robotiq_c_model_demo')
nodehandle = ros.NodeHandle()

grippers = {}

RobotiqCModel = torch.class('grippers.RobotiqCModel', grippers)

local function clamp(x, lo, hi)
  if x < lo then
    return lo
  elseif x > hi then
    return hi
  else
    return x
  end
end

local CModel_robot_input_spec = ros.MsgSpec('robotiq_c_model_control/CModel_robot_input')
local CModel_robot_output_spec = ros.MsgSpec('robotiq_c_model_control/CModel_robot_output')


local InitStatus = {
  Reset = 0,                  -- Gripper reset.
  Activation = 1              -- Gripper activation.
}

local ActionStatus = {
  Stopped = 0,                -- Stopped (or performing activation / automatic release).
  GoToPosition = 1            -- Go to Position Request.
}

local GripperStatus = {
  Reset = 0,                  -- Gripper is in reset ( or automatic release ) state. See Fault Status if Gripper is activated.
  Activating = 1,             -- Activation in progress.
  Activated = 3               -- Activation is completed.
}

local ObjStatus = {
  Moving            = 0,      -- Fingers are in motion towards requested position. No object detected.
  DetectedOpening   = 1,      -- Fingers have stopped due to a contact while opening before requested position. Object detected opening.
  DetectedClosing   = 2,      -- Fingers have stopped due to a contact while closing before requested position. Object detected closing.
  NoObject          = 3       -- Fingers are at requested position. No object detected or object has been loss / dropped.
}

local FaultStatus = {
  NoFault          = 0x00,    -- No fault (LED is blue)

                              -- Priority faults (LED is blue)
  ActionDelayed    = 0x052,   --   Action delayed, activation (reactivation) must be completed prior to renewed action.
  Deactivated      = 0x07,    --   The activation bit must be set prior to action.

                              -- Minor faults (LED continuous red)
  HighTemp         = 0x08,    --   Maximum operating temperature exceeded, wait for cool-down.

                              -- Major faults (LED blinking red/blue) - Reset is required (rising edge on activation bit rACT needed).
  LowVoltage       = 0x0A,    --   Under minimum operating voltage.
  AutoReleaseBusy  = 0x0B,    --   Automatic release in progress.
  CPUFault         = 0x0C,    --   Internal processor fault.
  ActivationFault  = 0x0D,    --   Activation fault, verify that no interference or other error occurred.
  Overcurrent      = 0x0E,    --   Overcurrent triggered.
  AutoReleaseDone  = 0x0F     --   Automatic release completed.
}

function RobotiqCModel:__init(nodehandle)
  self.nodehandle = nodehandle
  self.input = self.nodehandle:subscribe("/CModelRobotInput", CModel_robot_input_spec, 100)
  self.last_state = ros.Message(CModel_robot_input_spec)
  self.publisher = nodehandle:advertise("/CModelRobotOutput", CModel_robot_output_spec, 100)
  if not self.publisher:waitForSubscriber(1, 5) then
    error('Timeout: No subscription for CModelRobotOutput.')
  end
  self:reset()
end

function RobotiqCModel:shutdown()
  if self.input then
    self.input:shutdown()
    self.input = nil
  end
end

function RobotiqCModel:reset()
  self.cmd = ros.Message(CModel_robot_output_spec)
  self.cmd.rACT = 0
  self.publisher:publish(self.cmd)
end

function RobotiqCModel:publish()
  self.publisher:publish(self.cmd)
end

function RobotiqCModel:activate()
  self.cmd = ros.Message(CModel_robot_output_spec)
  self.cmd.rACT = 1
  self.cmd.rGTO = 1
  self.cmd.rSP  = 255
  self.cmd.rFR  = 10
  self:publish()
end

function RobotiqCModel:setSpeed(speed)
  self.cmd.rSP = clamp(speed, 0, 255)
  self:publish()
end

function RobotiqCModel:setForce(force)
  self.cmd.rFR = clamp(force, 0, 255)
  self:publish()
end

function RobotiqCModel:setPosition(pos)
  self.cmd.rPR = clamp(pos, 0, 255)
  self:publish()
end

function RobotiqCModel:open()
  self:setPosition(0)
end

function RobotiqCModel:close()
  self:setPosition(255)
end

function RobotiqCModel:spin()
  -- process available messages
  while self.input:hasMessage() do
    self.input:read(100, self.last_state)
  end
end

function RobotiqCModel:__tostring()
  return tostring(self.last_state)
end

spinner = ros.AsyncSpinner()
spinner:start()

gripper = grippers.RobotiqCModel(nodehandle)

gripper:reset()
gripper:activate()
gripper:setSpeed(5)
gripper:setForce(0)
ros.spinOnce()
gripper:open()
ros.spinOnce()
sys.sleep(2)
gripper:close()
ros.spinOnce()
sys.sleep(2)
gripper:shutdown()
