print '==> loading model from disk:'
model = torch.load(model_filename)

print("==> Measuring test dataset performance:")

te_crit_error = torch.FloatTensor(math.ceil(testData:size()/batch_size))
te_sample = 1
next_disp = math.max(batch_size, 100)
for t=1,testData:size(),batch_size do
  -- disp progress
  if (t >= next_disp) then
    progress(t, testData:size())
    next_disp = t + math.max(batch_size, 100)
  end
  
  -- create mini batch
  local cur_batch_start = t
  local cur_batch_end = math.min(t + batch_size - 1, testData:size())
  local cur_batch_size = cur_batch_end - cur_batch_start + 1
  local batchData = {
    files = {},
    data = {},
    -- labels = torch.CudaTensor(cur_batch_size, num_coeff),
    size = function() return cur_batch_size end,
    heat_maps = torch.FloatTensor(cur_batch_size, num_features * heat_map_height * 
      heat_map_width)
  }
  for j=1,num_hpf_banks do
    table.insert(batchData.data, torch.FloatTensor(cur_batch_size, 1, 
      bank_dim[j][1], bank_dim[j][2]))
  end
  
  local out_i = 1
  for i = cur_batch_start,cur_batch_end do
    -- Collect the current image and put it into the data slot
    for j=1,num_hpf_banks do
      batchData.data[j][{out_i,{},{},{}}] = testData.data[j][{i,{},{},{}}]
    end
    -- batchData.labels[{out_i,{}}] = testData.labels[i]
    batchData.heat_maps[{out_i,{}}] = torch.FloatTensor(
      testData.heat_maps[{i,{},{},{}}], 1, torch.LongStorage{num_features *
      heat_map_height * heat_map_width})
    out_i = out_i + 1
  end
  for j=1,num_hpf_banks do
    batchData.data[j] = batchData.data[j]:cuda()
  end
  batchData.heat_maps = batchData.heat_maps:cuda()

  pred = model:forward(batchData.data)
  te_crit_error[te_sample] = math.abs(criterion:forward(pred, 
    batchData.heat_maps))

  err = te_crit_error[te_sample]
  if (err == math.huge or err ~= err) then
    print(string.format("nan or inf found!\n"))
  end

  te_sample = te_sample + 1
end
te_sample = te_sample - 1
progress(testData:size(), testData:size())

print '==> Measuring training dataset performance:'

tr_crit_error = torch.FloatTensor(math.ceil(trainData:size()/batch_size))
tr_sample = 1
next_disp = math.max(batch_size, 100)
for t=1,trainData:size(),batch_size do
  -- disp progress
  if (t >= next_disp) then
    progress(t, trainData:size())
    next_disp = next_disp + math.max(batch_size, 100)
  end

  -- create mini batch
  local cur_batch_start = t
  local cur_batch_end = math.min(t + batch_size - 1, trainData:size())
  local cur_batch_size = cur_batch_end - cur_batch_start + 1
  local batchData = {
    files = {},
    data = {},
    -- labels = torch.CudaTensor(cur_batch_size, num_coeff),
    size = function() return cur_batch_size end,
    heat_maps = torch.FloatTensor(cur_batch_size, num_features * heat_map_height * 
        heat_map_width)
  }
  for j=1,num_hpf_banks do
    table.insert(batchData.data, torch.FloatTensor(cur_batch_size, 1,
      bank_dim[j][1], bank_dim[j][2]))
  end
  local out_i = 1
  for i = cur_batch_start,cur_batch_end do
    -- Collect the current image and put it into the data slot
    for j=1,num_hpf_banks do
      batchData.data[j][{out_i,{},{},{}}] = trainData.data[j][{i,{},{},{}}]
    end
    -- batchData.labels[{out_i,{}}] = trainData.labels[i]
    batchData.heat_maps[{out_i,{}}] = torch.FloatTensor(
      trainData.heat_maps[{i,{},{},{}}], 1, torch.LongStorage{num_features *
      heat_map_height * heat_map_width})
    out_i = out_i + 1
  end
  for j=1,num_hpf_banks do
    batchData.data[j] = batchData.data[j]:cuda()
  end
  batchData.heat_maps = batchData.heat_maps:cuda()

  pred = model:forward(batchData.data)
  tr_crit_error[tr_sample] = math.abs(criterion:forward(pred,
    batchData.heat_maps))

  err = tr_crit_error[tr_sample]
  if (err == math.huge or err ~= err) then
    print(string.format("nan or inf found!\n"))
  end

  tr_sample = tr_sample + 1
end
tr_sample = tr_sample - 1
progress(trainData:size(), trainData:size())

print '    Average Training Set Error Value:'
print(tr_crit_error:mean())

print '    Average Test Set Error Value:'
print(te_crit_error:mean())

-- ********************** Show the heat map of an output ***********************
trImage_index = 410
print(string.format('Training set image: %d', trImage_index))
trImage = {
  files = {},
  data = {},
  labels = trainData.labels[{{trImage_index},{}}],
  size = function() return 1 end,
  heat_maps = trainData.heat_maps[{{trImage_index},{},{},{}}]
}
for j=1,num_hpf_banks do
  table.insert(trImage.data, trainData.data[j][{{trImage_index},{},{},{}}])
end
dofile('visualize_data.lua')  -- Just in case it hasn't been loaded
--VisualizeImage(trImage, 1)
for j=1,num_hpf_banks do
  trImage.data[j] = trImage.data[j]:cuda()
end
target_heat_map_vec = torch.FloatTensor(
  trImage.heat_maps, 1, torch.LongStorage{num_features *
  heat_map_height * heat_map_width})
trImage.heat_maps = model:forward(trImage.data):float()
for j=1,num_hpf_banks do
  trImage.data[j] = trImage.data[j]:float()
end
VisualizeImage(trImage, 1)

teImage_index = 410
print(string.format('Test set image: %d', teImage_index))
teImage = {
  files = {},
  data = {},
  labels = testData.labels[{{teImage_index},{}}],
  size = function() return 1 end,
  heat_maps = testData.heat_maps[{{teImage_index},{},{},{}}]
}
for j=1,num_hpf_banks do
  table.insert(teImage.data, testData.data[j][{{teImage_index},{},{},{}}])
end
VisualizeImage(teImage, 1)
for j=1,num_hpf_banks do
  teImage.data[j] = teImage.data[j]:cuda()
end
teImage.heat_maps = model:forward(teImage.data):float()
for j=1,num_hpf_banks do
  teImage.data[j] = teImage.data[j]:float()
end
VisualizeImage(teImage, 1)





