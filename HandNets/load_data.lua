

-- ************ Get Data locations ***************
print '==> Scanning directory for hand data...'
if paths.dirp(im_dir) == false then
  print("Couldn't find image directory")
  return
else 
  -- Collect the filenames
  files = {}
  -- THIS METHOD WONT RETURN THEM SORTED (which takes a long time to do)
  -- for f in paths.files(im_dir) do
  --   files[i] = f
  --   i = i+1
  -- end
  -- THIS METHOD RETURNS A SORTED LIST
  for f in io.popen("ls " .. im_dir):lines() do
    table.insert(files, f)
  end
end
-- Partition files into their respective groups
-- coeffl_files = {}
coeffr_files = {}
hpf_depth_files = {}
depth_files = {}
for i=1,#files,1 do
  if files[i] ~= nil then
    if string.find(files[i], "coeffr_") ~= nil then
      table.insert(coeffr_files, files[i])
--    elseif string.find(files[i], "coeffl_hands_") ~= nil then
--      table.insert(coeffl_files, files[i])
    elseif string.find(files[i], "hpf_processed_") ~= nil then
      table.insert(hpf_depth_files, files[i])
    elseif string.find(files[i], "processed_") ~= nil then
      table.insert(depth_files, files[i])
    end
  end
end 

if paths.dirp(test_im_dir) == false then
  print("Couldn't find test image directory")
  return
else 
  -- Collect the filenames
  files = {}
  for f in io.popen("ls " .. test_im_dir):lines() do
    table.insert(files, f)
  end
end
-- Partition files into their respective groups
-- coeffl_files = {}
test_coeffr_files = {}
test_hpf_depth_files = {}
test_depth_files = {}
for i=1,#files,1 do
  if files[i] ~= nil then
    if string.find(files[i], "coeffr_") ~= nil then
      table.insert(test_coeffr_files, files[i])
--    elseif string.find(files[i], "coeffl_hands_") ~= nil then
--      table.insert(coeffl_files, files[i])
    elseif string.find(files[i], "hpf_processed_") ~= nil then
      table.insert(test_hpf_depth_files, files[i])
    elseif string.find(files[i], "processed_") ~= nil then
      table.insert(test_depth_files, files[i])
    end
  end
end 

if (use_hpf_depth == 1) then
  im_files = hpf_depth_files
  test_im_files = test_hpf_depth_files
else
  im_files = depth_files
  test_im_files = test_depth_files

end
depth_files = nil
test_depth_files = nil
hpf_depth_files = nil
test_hpf_depth_files = nil

-- EDIT: NO NEED TO SORT NOW THAT ls METHOD IS RETURNING SORTED ORDER
-- print("==> Sorting files so indicies match...")
-- function stringComp (a, b) 
--   return string.lower(a) < string.lower(b) 
-- end
-- table.sort(im_files, stringComp)
-- table.sort(test_im_files, stringComp)
-- table.sort(coeffr_files, stringComp)
-- table.sort(test_coeffr_files, stringComp)

-- ************ Randomly permute the files ***********
print("==> Permuting the files...")
shuffle_files_right(coeffr_files, im_files)
shuffle_files_right(test_coeffr_files, test_im_files)

-- ************ Check the file order ***********
print("==> Verifying the file order...")
for i=1,#im_files do
  local tFinal = {}
  funca = string.gfind(im_files[i], "(%d+)")  -- Returns an iterator that will parse each number
  funcb = string.gfind(coeffr_files[i], "(%d+)")
  -- All numbers should match
  repeat
    numa = funca()
    numb = funcb()
    if (numa ~= numb) then
      print("Image file " .. string.format("%d", i) .. " doesn't match coeff file number!")
      return
    end
  until numa == nil
end
for i=1,#test_im_files do
  local tFinal = {}
  funca = string.gfind(test_im_files[i], "(%d+)")  -- Returns an iterator that will parse each number
  funcb = string.gfind(test_coeffr_files[i], "(%d+)")
  -- All numbers should match
  repeat
    numa = funca()
    numb = funcb()
    if (numa ~= numb) then
      print("Image file " .. string.format("%d", i) .. " doesn't match coeff file number!")
      return
    end
  until numa == nil
end

-- ************ Load data from Disk ***************
print '==> Loading hand data from directory...'
trsize = #im_files
tesize = #test_im_files
trainData = {
  files = {},
  data = {},  -- Multi-bank input (usually 3)
  labels = torch.FloatTensor(trsize, num_coeff),
  heat_maps = nil,
  size = function() return trsize end
}
testData = {
  files = {},
  data = {},
  labels = torch.FloatTensor(tesize, num_coeff),
  heat_maps = nil,
  size = function() return tesize end
}

for i=1,num_hpf_banks do
  table.insert(trainData.data, torch.FloatTensor(trsize, 1, 
    bank_dim[i][1], bank_dim[i][2]))
  table.insert(testData.data, torch.FloatTensor(tesize, 1, 
    bank_dim[i][1], bank_dim[i][2]))
end

-- LOAD IN TRAINING SET DIRECTORY --> SOME IMAGES GO TO TEST SET AS WELL!
itr = 1
for i=1,#im_files do
  -- disp progress
  if (math.mod(i, 1000) == 1 or i == #im_files) then
    progress(i, #im_files)
  end

  -- Read in the sample
  coeff_file = torch.DiskFile(im_dir .. coeffr_files[i], 'r')
  coeff_file:binary()
  coeff_data = coeff_file:readFloat(num_coeff)
  coeff_file:close()

  hpf_depth_file = torch.DiskFile(im_dir .. im_files[i],'r')
  hpf_depth_file:binary()
  hpf_depth_data = hpf_depth_file:readFloat(data_file_size)
  hpf_depth_file:close()

  -- this sample is training data
  -- We need to split the long vector
  ind = 1
  for j=1,num_hpf_banks do
    trainData.data[j][{itr, 1, {}, {}}] = torch.FloatTensor(
      hpf_depth_data, ind, torch.LongStorage{bank_dim[j][1], 
      bank_dim[j][2]}):float()
    ind = ind + (bank_dim[j][1]*bank_dim[j][2]) -- Move pointer forward
  end
  trainData.labels[{itr, {}}] = torch.FloatTensor(coeff_data, 1,
    torch.LongStorage{num_coeff}):float()
  trainData.files[itr] = im_files[i]
  itr = itr + 1
end
itr = itr - 1

-- NOW LOAD IN TEST SET DIRECTORY
ite = 1
for i=1,#test_im_files do
  -- disp progress
  if (math.mod(i, 100) == 1 or i == #test_im_files) then
    progress(i, #test_im_files)
  end

  -- Read in the sample
  coeff_file = torch.DiskFile(test_im_dir .. test_coeffr_files[i], 'r')
  coeff_file:binary()
  coeff_data = coeff_file:readFloat(num_coeff)
  coeff_file:close()

  hpf_depth_file = torch.DiskFile(test_im_dir .. test_im_files[i],'r')
  hpf_depth_file:binary()
  hpf_depth_data = hpf_depth_file:readFloat(data_file_size)
  hpf_depth_file:close()

  -- this sample is test data
  -- We need to split the long vector
  ind = 1
  for j=1,num_hpf_banks do
    testData.data[j][{ite, 1, {}, {}}] = torch.FloatTensor(
      hpf_depth_data, ind, torch.LongStorage{bank_dim[j][1], 
      bank_dim[j][2]})
    ind = ind + (bank_dim[j][1]*bank_dim[j][2]) -- Move pointer forward
  end
  testData.labels[{ite, {}}] = torch.FloatTensor(coeff_data, 1,
    torch.LongStorage{num_coeff}):float()
  testData.files[ite] = test_im_files[i]
  ite = ite + 1
end
ite = ite - 1

print(string.format("    Loaded %d test set images and %d training set images", 
  tesize, trsize))

-- Force a return (just load data)
-- if (true) then
--   return
-- end

-- ********************** Converting data to cuda **********************
if false then
  print '==> Converting image data to cudaTensor'
  for k=1,num_hpf_banks do
    testData.data[k] = testData.data[k]:cuda()
  end
  for k=1,num_hpf_banks do
    trainData.data[k] = trainData.data[k]:cuda()
  end
end
if false then
  print '==> Converting label data to cudaTensor'
  trainData.labels = trainData.labels:cuda()
  testData.labels = testData.labels:cuda()
end

-- ***************** Convert label data to heat maps ********************
dofile("centered_gaussian.lua")
-- Test the centered gaussian
-- im = image.centered_gaussian{sigma=(1/width), amplitude=1, normalize=true, width=96, height=96, center_x=0.75, center_y=0.25}
-- image.display{image=im, zoom=5}
-- Generate the heat maps
testData.heat_maps = torch.FloatTensor(testData:size(), num_features, heat_map_height, 
  heat_map_width)
trainData.heat_maps = torch.FloatTensor(trainData:size(), num_features, heat_map_height, 
  heat_map_width)
if (regenerate_heat_maps == 1) then
  os.execute('mkdir -p ' .. heatmap_dir)
  cur_heat_map = torch.FloatTensor(num_features, heat_map_height, heat_map_width)

  print '==> Creating heat map images on test set'
  for i=1,testData:size() do
    -- disp progress
    if (math.mod(i, 100) == 1 or i == testData:size()) then
      progress(i, testData:size())
    end

    local f
    f = 1
    for c=1,num_coeff,num_coeff_per_feature do
      cur_uv = testData.labels[{i,{c, c + 1}}]
      cur_heat_map[{f,{},{}}] = image.centered_gaussian{amplitude=1,
        normalize=true, width=heat_map_width, height=heat_map_height, center_x=cur_uv[1], 
        center_y=cur_uv[2], sigma_horz=(heat_map_sigma/heat_map_width), sigma_vert=(heat_map_sigma/heat_map_height)}
      f = f + 1
    end
    -- cur_heat_map:add(-cur_heat_map:mean())
    cur_heat_map:div(cur_heat_map:std())
    --[[
    filename = heatmap_dir .. 'heatmap_' .. testData.files[i]
    heatmap_file = torch.DiskFile(filename, 'w')
    heatmap_file:binary()
    heatmap_file:writeFloat(cur_heat_map:storage())
    heatmap_file:close()
    --]]

    testData.heat_maps[{i,{},{},{}}]:copy(cur_heat_map)
  end

  print '==> Creating heat map images on training set'
  for i=1,trainData:size() do
    -- disp progress
    if (math.mod(i, 1000) == 1 or i == trainData:size()) then
      progress(i, trainData:size())
    end

    local f
    f = 1
    for c=1,num_coeff,num_coeff_per_feature do
      cur_uv = trainData.labels[{i,{c, c + 1}}]
      cur_heat_map[{f,{},{}}] = image.centered_gaussian{amplitude=1,
        normalize=true, width=heat_map_width, height=heat_map_height, center_x=cur_uv[1], 
        center_y=cur_uv[2], sigma_horz=(heat_map_sigma/heat_map_width), sigma_vert=(heat_map_sigma/heat_map_height)}
      f = f + 1
    end
    -- cur_heat_map:add(-cur_heat_map:mean())
    cur_heat_map:div(cur_heat_map:std())
    --[[
    filename = heatmap_dir .. 'heatmap_' .. trainData.files[i]
    heatmap_file = torch.DiskFile(filename, 'w')
    heatmap_file:binary()
    heatmap_file:writeFloat(cur_heat_map:storage())
    heatmap_file:close()
    --]]

    trainData.heat_maps[{i,{},{},{}}]:copy(cur_heat_map)
  end
else
  print '==> Loading test set heat maps from file'
--[[
  for i=1,testData:size() do
    -- disp progress
    if (math.mod(i, 100) == 1 or i == testData:size()) then
      progress(i, testData:size())
    end
    
    filename = heatmap_dir .. 'heatmap_' .. testData.files[i]
    heatmap_file = torch.DiskFile(filename, 'r')
    heatmap_file:binary()
    heatmap_data = heatmap_file:readFloat(num_features * heat_map_width * heat_map_height)
    heatmap_file:close()
  
    testData.heat_maps[{i,{},{},{}}]:copy(torch.FloatTensor(heatmap_data, 1,
      torch.LongStorage{num_features, heat_map_width, heat_map_height}):float())
  end
  print '==> Loading training set heat maps from file'
  for i=1,trainData:size() do
    -- disp progress
    if (math.mod(i, 1000) == 1 or i == trainData:size()) then
      progress(i, trainData:size())
    end

    filename = heatmap_dir .. 'heatmap_' .. trainData.files[i]
    heatmap_file = torch.DiskFile(filename, 'r')
    heatmap_file:binary()
    heatmap_data = heatmap_file:readFloat(num_features * heat_map_width * heat_map_height)
    heatmap_file:close()
  
    trainData.heat_maps[{i,{},{},{}}]:copy(torch.FloatTensor(heatmap_data, 1,
      torch.LongStorage{num_features, heat_map_width, heat_map_height}):float())
  end
--]]
  print("ERROR: Heatmaps from disk might be broken!")
  error("ERROR: Heatmaps from disk might be broken!")
end  -- if (regenerate_heat_maps == 1) then






