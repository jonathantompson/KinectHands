-- THIS IS THE HELPER FUNCTION FOR PRETURB.LUA

-- preturbDataLabels --> Helper function.  Rotates one image only.
-- if in_* is not se (or is nil), then a random value is chosen (between min and max with uniform dist)
function preturbDataAndLabels(im, labels, heat_maps, in_deg_rot, in_scale, in_transv, in_transu)
  local deg_rot_bounds = 60
  local scale_bounds = 0.1
  local trans_bounds = 0.1
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
  local im_out
  local deg_rot  -- angle in degrees
  local scale  -- width' = is scale_val * width, etc
  local transv  -- center_y' = center_y + transv * height
  local transu  -- center_x' = center_x + transv * width
  local rotmat
  im_out, deg_rot, scale, transv, transu, rotmat = distort(im_pad, deg_rot_bounds, 
    scale_bounds, trans_bounds, in_deg_rot, in_scale, in_transv, in_transu)
  im_out = im_out[{{}, {2,1 + im:size()[2]}, {2,1 + im:size()[3]}}]

  -- Rotate the labels:
  --[[
  local labels_out = nil
  if (labels ~= nil) then
    labels_out = labels:clone():float()  -- 0 to 1
    labels_out:mul(2)
    labels_out:add(-1) -- Now -1 to 1
    for i=1,num_coeff, num_coeff_per_feature do
      labels_out[{i}] = labels_out[{i}] * (1.0 - scale)
      labels_out[{i+1}] = labels_out[{i+1}] * (1.0 - scale)
      labels_out[{i}] = (labels_out[{i}] - 2.0 * transu)
      labels_out[{i+1}] = (labels_out[{i+1}] - 2.0 * transv)
      local vec = labels_out[{{i,i+1}}]
      labels_out[{{i,i+1}}] = torch.mv(rotmat, vec)
    end
    labels_out:add(1)
    labels_out:mul(0.5)
  end
  --]]
  -- EDIT: MANUAL LABEL ROTATION DOESN'T WORK, SO JUST COPY THEM
  local labels_out = nil
  if (labels ~= nil) then
    labels_out = labels:clone():float()
  end

  -- Rotate the heat maps:
  local heat_maps_out = nil
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
    heat_maps_out = distort(heat_maps_pad, deg_rot_bounds, scale_bounds, trans_bounds, 
      deg_rot, scale, transv, transu)
    heat_maps_out = heat_maps_out[{{}, {2,1 + heat_maps:size()[2]}, {2,1 + heat_maps:size()[3]}}]
  end
  

  return im_out, labels_out, heat_maps_out, deg_rot, scale, transv, transu
end

