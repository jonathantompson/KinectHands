-- THIS IS THE HELPER FUNCTION FOR PRETURB.LUA

-- preturbDataLabels --> Helper function.  Rotates one image only.
-- if deg_rot is not set, then a random rotation is chosen (between min and max with uniform dist)
function preturbDataAndLabels(im, labels, heat_maps, deg_rot, shift_frac_u, shift_frac_v)
  local min_max_rotation_deg = 60
  if (im:dim() ~= 3) then
    error("im dimension is not 3!")
  end
  -- allocate some temporary space
  if (im_pad ~= nil) then
    if (im_pad:size()[1] ~= im:size()[1] or
      im_pad:size()[2] ~= im:size()[2]+2 or
      im_pad:size()[3] ~= im:size()[3]+2) then
      im_pad = nil  -- for a resize of the variable
    end
  end
  im_pad = im_pad or torch.zeros(im:size()[1], im:size()[2]+2, im:size()[3]+2):float()
  im_pad[{{},{2,1+im:size()[2]}, {2,1+im:size()[3]}}] = im

  -- Rotate the image
  local im_rotate
  local rotation_angle
  local mat
  im_rotate, rotation_angle, mat = distort(im_pad, 0.0, 0.0, 1.0, min_max_rotation_deg, deg_rot)
  im_rotate = im_rotate[{{}, {2,1 + im:size()[2]}, {2,1 + im:size()[3]}}]

  -- Rotate the labels:
  local labels_rotate = nil
  if (labels ~= nil) then
    labels_rotate = labels:clone():float()
    labels_rotate:mul(2)
    labels_rotate:add(-1) -- Now -1 to 1
    for i=1,num_coeff,2 do
      local vec = labels_rotate[{{i,i+1}}]
      labels_rotate[{{i,i+1}}] = torch.mv(mat, vec)
    end
    labels_rotate:add(1)
    labels_rotate:mul(0.5)
  end

  -- Rotate the heat maps:
  local heat_maps_rotate = nil
  if (heat_maps ~= nil) then
    
    if (heat_maps_pad ~= nil) then
      if (heat_maps_pad:size()[1] ~= heat_maps:size()[1] or
        heat_maps_pad:size()[2] ~= heat_maps:size()[2]+2 or
        heat_maps_pad:size()[3] ~= heat_maps:size()[3]+2) then
        heat_maps_pad = nil  -- for a resize of the variable
      end
    end
    heat_maps_pad = heat_maps_pad or torch.zeros(heat_maps:size()[1], 
      heat_maps:size()[2]+2, heat_maps:size()[3]+2):float()
    heat_maps_pad[{{},{2,1+heat_maps:size()[2]}, {2,1+heat_maps:size()[3]}}] = heat_maps
    heat_maps_rotate = distort(heat_maps_pad, 0.0, 0.0, 1.0, min_max_rotation_deg, rotation_angle)
    heat_maps_rotate = heat_maps_rotate[{{}, {2,1 + heat_maps:size()[2]}, {2,1 + heat_maps:size()[3]}}]
  end

  -- Now randomly shift the image
  if (shift_frac_u == nil or shift_frac_v == nil) then
    local min_max_shift_u = 0.2 -- Percentage of image
    local min_max_shift_v = 0.2 -- Percentage of image
    local shift_frac_u = torch.uniform(-min_max_shift_u, min_max_shift_u)
    local shift_frac_v = torch.uniform(-min_max_shift_v, min_max_shift_v)
  end
  

  return im_rotate, labels_rotate, heat_maps_rotate, rotation_angle, shift_frac_u, shift_frac_v
end

