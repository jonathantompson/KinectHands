require 'image'

-- distort function
-- With probabilities, distort will flip, scale and rotate the image
-- The rotation angle is a random value between:
--   -deg_rot_bounds --> deg_rot_bounds
-- To force a certain rotation, set deg_rot to some non-nil value
function distort(i, deg_rot_bounds, scale_bounds, trans_bounds, deg_rot, scale, trans_v, trans_u)
   deg_rot_bounds = deg_rot_bounds or 10
   trans_bounds = trans_bounds or 0.2
   scale_bounds = scale_bounds or 0.1
   -- size:
   local height,width = i:size(2),i:size(3)

   -- x/y grids
   local grid_y = torch.ger( torch.linspace(-1,1,height), torch.ones(width) )
   local grid_x = torch.ger( torch.ones(height), torch.linspace(-1,1,width) )

   local flow = torch.FloatTensor()
   local flow_scale = torch.FloatTensor()
   local flow_rot = torch.FloatTensor()

   -- global flow:
   flow:resize(2,height,width)
   flow:zero()

   local rot_angle
   local rotmat

   -- Apply scale
   flow_scale:resize(2,height,width)
   flow_scale[1] = grid_y
   flow_scale[2] = grid_x
   scale_val = scale or torch.uniform(-scale_bounds, scale_bounds)
   flow_scale[1]:mul(scale_val * height)
   flow_scale[2]:mul(scale_val * width)
   flow:add(flow_scale)

   -- Apply translation (comes before rotation)
   local trans_v_val = trans_v or torch.uniform(-trans_bounds, trans_bounds)
   local trans_u_val = trans_u or torch.uniform(-trans_bounds, trans_bounds)
   flow[1]:add(trans_v_val * height)
   flow[2]:add(trans_u_val * width)

   -- Apply rotation
   flow_rot:resize(2,height,width)
   flow_rot[1] = grid_y * ((height-1)/2) * -1
   flow_rot[2] = grid_x * ((width-1)/2) * -1
   local view = flow_rot:reshape(2,height*width)
   local function rmat(deg)
   local r = deg/180*math.pi
   return torch.FloatTensor{{math.cos(r), -math.sin(r)}, 
                                  {math.sin(r), math.cos(r)}}
   end
      
   rot_angle = deg_rot or torch.uniform(-deg_rot_bounds,deg_rot_bounds)
   rotmat = rmat(rot_angle)
   flow_rotr = torch.mm(rotmat, view)
   flow_rot = flow_rot - flow_rotr:reshape( 2, height, width )
   flow:add(flow_rot)

   -- apply field
   local result = torch.FloatTensor()
   image.warp(result,i,flow,'bilinear')
   return result, rot_angle, scale_val, trans_v_val, trans_u_val, rotmat 
end

-- local function test()
--    local i = image.lena():float()
--    i = image.scale(i, 128, 128)
--    i = distort(i, 0.25, 0.5, 0.75)
---   image.display(i)
-- end
--- for t = 1,10 do test() end

-- return distort

