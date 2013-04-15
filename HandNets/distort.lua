
require 'image'

-- distort function
flow,flow1,flow2,flow3,result
   = torch.FloatTensor(),torch.FloatTensor(),
     torch.FloatTensor(),torch.FloatTensor(),
     torch.FloatTensor()

-- With probabilities, distort will flip, scale and rotate the image
-- The rotation angle is a random value between:
--   -deg_rot_bounds --> deg_rot_bounds
-- To force a certain rotation, set deg_rot to some non-nil value
function distort(i, prob_flip, prob_scale, prob_rot, deg_rot_bounds, deg_rot)
   deg_rot_bounds = deg_rot_bounds or 10
   -- size:
   local height,width = i:size(2),i:size(3)

   -- x/y grids
   local grid_y = torch.ger( torch.linspace(-1,1,height), torch.ones(width) )
   local grid_x = torch.ger( torch.ones(height), torch.linspace(-1,1,width) )

   -- global flow:
   flow:resize(2,height,width)
   flow:zero()

   local rot_angle
   local rotmat

   --[[
   -- flip
   if 1 == torch.bernoulli(prob_flip) then
      flow1:resize(2,height,width)
      flow1[1] = 0
      flow1[2] = grid_x
      flow1[2]:mul(-width)
      flow:add(flow1)

   -- scale field
   elseif 1 == torch.bernoulli(prob_scale) then
      flow2:resize(2,height,width)
      flow2[1] = grid_y
      flow2[2] = grid_x
      flow2[1]:mul(torch.uniform(-height/10,height/10))
      flow2[2]:mul(torch.uniform(-width/10,width/10))
      flow:add(flow2)
   
   -- rotation field
   elseif 1 == torch.bernoulli(prob_rot) then
      flow3:resize(2,height,width)
      flow3[1] = grid_y * ((height-1)/2) * -1
      flow3[2] = grid_x * ((width-1)/2) * -1
      local view = flow3:reshape(2,height*width)
      local function rmat(deg)
         local r = deg/180*math.pi
         return torch.FloatTensor{{math.cos(r), -math.sin(r)}, 
                                  {math.sin(r), math.cos(r)}}
      end
      
      rot_angle = deg_rot or torch.uniform(-deg_rot_bounds,deg_rot_bounds)
      rotmat = rmat(rot_angle)
      flow3r = torch.mm(rotmat, view)
      flow3 = flow3 - flow3r:reshape( 2, height, width )
      flow:add(flow3)
   end
   --]]
   flow3:resize(2,height,width)
   flow3[1] = grid_y * ((height-1)/2) * -1
   flow3[2] = grid_x * ((width-1)/2) * -1
   local view = flow3:reshape(2,height*width)
   local function rmat(deg)
   local r = deg/180*math.pi
   return torch.FloatTensor{{math.cos(r), -math.sin(r)}, 
                                  {math.sin(r), math.cos(r)}}
   end
      
   rot_angle = deg_rot or torch.uniform(-deg_rot_bounds,deg_rot_bounds)
   rotmat = rmat(rot_angle)
   flow3r = torch.mm(rotmat, view)
   flow3 = flow3 - flow3r:reshape( 2, height, width )
   flow:add(flow3)

   -- apply field
   image.warp(result,i,flow,'bilinear')
   return result, rot_angle, rotmat
end

-- local function test()
--    local i = image.lena():float()
--    i = image.scale(i, 128, 128)
--    i = distort(i, 0.25, 0.5, 0.75)
---   image.display(i)
-- end
--- for t = 1,10 do test() end

-- return distort

