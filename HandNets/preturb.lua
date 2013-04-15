dofile("distort.lua")

-- preturbDataLabels --> Helper function.  Rotates one image only.
-- if deg_rot is not set, then a random rotation is chosen (between min and max with uniform dist)
function preturbDataAndLabels(im, labels, deg_rot, shift_frac_u, shift_frac_v)
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

  -- Now randomly shift the image
  if (shift_frac_u == nil or shift_frac_v == nil) then
    local min_max_shift_u = 0.2 -- Percentage of image
    local min_max_shift_v = 0.2 -- Percentage of image
    local shift_frac_u = torch.uniform(-min_max_shift_u, min_max_shift_u)
    local shift_frac_v = torch.uniform(-min_max_shift_v, min_max_shift_v)
  end
  

  return im_rotate, labels_rotate, rotation_angle, shift_frac_u, shift_frac_v
end

-- Top level function, preturbs the whole database
function preturb(database) 
  local data_rotated = {
    data = { },
    labels = database.labels:clone():float(),
    size = function() return database:size() end
  }
  for i=1,num_hpf_banks do
    table.insert(data_rotated.data, database.data[i]:clone():float())
  end

  -- Preturb the database
  local next_progress = 100
  for j=1,data_rotated:size() do
    -- disp progress
    if (j >= next_progress or j == data_rotated:size()) then
      -- progress(j, data_rotated:size())
      next_progress = next_progress + 100
    end

    -- rotate the first bank and get back the rotation matrix
    data_rotated.data[1][{j,{},{},{}}], data_rotated.labels[{j,{}}], 
      deg_rot, shift_u, shift_v = 
     preturbDataAndLabels(database.data[1][{j,{},{},{}}], database.labels[{j,{}}])
    -- Now rotate the other banks by the same transformation
    for b=2,num_hpf_banks do
      data_rotated.data[b][{j,{},{},{}}] = 
        preturbDataAndLabels(database.data[b][{j,{},{},{}}], nil, deg_rot, shift_u, shift_v)
    end
  end
  
  return data_rotated
end
