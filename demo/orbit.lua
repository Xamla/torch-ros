ros = require 'ros'
tf = ros.tf

--[[

Compute points along an arc on a plane around a center point and normal.

You can use rviz to display the moving 'eye' transform with the TF display.
Please make sure 'Fixed Frame' is set to 'world'.

]]

function normalize(v)
  return v / torch.norm(v)
end

function totensor(t)
  return not t or torch.isTensor(t) and t or torch.Tensor(t)
end

function pos_vector(v)
  if v:size(1) ~= 4 then
    v = torch.Tensor({ v[1], v[2], v[3], 1 })
  end
  return v
end

function project_onto_plane(plane_point, plane_normal, pt)
  return pt - plane_normal * torch.dot(pt - plane_point, plane_normal)
end

function look_at_pose(eye, at, up)
  -- eye becomes origin, 'at' lies on z-axis
  local xaxis = normalize(at - eye)
  local yaxis = -normalize(torch.cross(xaxis, up))
  local zaxis = torch.cross(xaxis, yaxis)

  local basis = torch.Tensor(3,3)
  basis[{{},{1}}] = xaxis
  basis[{{},{2}}] = yaxis
  basis[{{},{3}}] = zaxis

  local t = tf.Transform()
  t:setBasis(basis)
  t:setOrigin(eye)
  return t
end

function generate_arc(center, normal, start_pt, total_rotation_angle, angle_step, look_at, up)
  up = up or torch.Tensor({0,0,1})
  look_at = look_at or center

  center = totensor(center)
  normal = totensor(normal)
  start_pt = totensor(start_pt)
  start_pt = project_onto_plane(center, normal, start_pt)
  look_at = totensor(look_at)
  up = totensor(up)

  local poses = {}
  local steps = math.max(math.floor(total_rotation_angle / angle_step + 0.5), 1)
  for i=0,steps do
    local theta = total_rotation_angle * i / steps

    local t = tf.Transform()
    t:setRotation(tf.Quaternion(normal, theta))
    t:setOrigin(center)

    local eye = t:toTensor() * pos_vector(start_pt-center)
    local pose = look_at_pose(eye[{{1,3}}], look_at, up)
    table.insert(poses, pose)
  end

  return poses
end

x = generate_arc({1,1,0.25}, {0,0,1}, {0.5,1.5,1.25}, 2 * math.pi, 0.1, {1,1,0})

ros.init('lookat')
ros.Time.init()

local b = tf.TransformBroadcaster()

local i = 1
while ros.ok() do
print(x[i])
  local st = tf.StampedTransform(x[i], ros.Time.now(), 'world', 'eye')
  b:sendTransform(st)
  ros.Duration(0.1):sleep()
  i = (i % #x) + 1
  ros.spinOnce()
end
