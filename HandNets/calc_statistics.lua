print '==> loading model from disk:'
model = torch.load(model_filename)

print("==> Measuring test dataset performance:")

te_crit_error = torch.FloatTensor(math.ceil(testData:size()/batch_size))
te_crit_err_by_coeff = torch.zeros(num_coeff):float()
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
    labels = torch.CudaTensor(cur_batch_size, num_coeff),
    size = function() return cur_batch_size end
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
    batchData.labels[{out_i,{}}] = testData.labels[i]
    out_i = out_i + 1
  end
  for j=1,num_hpf_banks do
    batchData.data[j] = batchData.data[j]:cuda()
  end

  pred = model:forward(batchData.data)
  te_crit_error[te_sample] = math.abs(criterion:forward(pred, 
    batchData.labels))

  err = te_crit_error[te_sample]
  if (err == math.huge or err ~= err) then
    print(string.format("nan or inf found!\n"))
  end
  
  pred = pred:float()
  batchData.labels = batchData.labels:float()
  delta = (pred - batchData.labels)
  delta:abs()
  
  for i=1,cur_batch_size do
    te_crit_err_by_coeff:add(delta[{i,{}}])
  end

  te_sample = te_sample + 1
end
te_sample = te_sample - 1
progress(testData:size(), testData:size())

print '==> Measuring training dataset performance:'

tr_crit_error = torch.FloatTensor(math.ceil(trainData:size()/batch_size))
tr_crit_err_by_coeff  = torch.zeros(num_coeff):float()
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
    labels = torch.CudaTensor(cur_batch_size, num_coeff),
    size = function() return cur_batch_size end
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
    batchData.labels[{out_i,{}}] = trainData.labels[i]
    out_i = out_i + 1
  end
  for j=1,num_hpf_banks do
    batchData.data[j] = batchData.data[j]:cuda()
  end

  pred = model:forward(batchData.data)
  tr_crit_error[tr_sample] = math.abs(criterion:forward(pred,
    batchData.labels))

  err = tr_crit_error[tr_sample]
  if (err == math.huge or err ~= err) then
    print(string.format("nan or inf found!\n"))
  end

  pred = pred:float()
  batchData.labels = batchData.labels:float()
  delta = (pred - batchData.labels)
  delta:abs()

  for i=1,cur_batch_size do
    tr_crit_err_by_coeff:add(delta[{i,{}}])
  end

  tr_sample = tr_sample + 1
end
tr_sample = tr_sample - 1
progress(trainData:size(), trainData:size())

tr_crit_err_by_coeff:mul(1 / trainData:size())
te_crit_err_by_coeff:mul(1 / testData:size()) 

print '==> Printing statistics:'

print '    Average Test Set abs(Error) Value BY COEFF:'
print(te_crit_err_by_coeff)
print '    Average Training Set abs(Error) Value BY COEFF:'
print(tr_crit_err_by_coeff)

print '    Average Training Set Error Value:'
print(tr_crit_error:mean())

print '    Average Test Set Error Value:'
print(te_crit_error:mean())

--tr_crit_error = torch.sort(tr_crit_error)
--te_crit_error = torch.sort(te_crit_error)

--print '    Average Training Set Error Value top 20%:'
--print(tr_crit_error[{{1,math.floor(0.2*trainData:size())}}]:mean())

--print '    Average Test Set Error Value top 20%:'
--print(te_crit_error[{{1,math.floor(0.2*testData:size())}}]:mean())

--print '    Average Training Set Error Value top 80%:'
--print(tr_crit_error[{{1,math.floor(0.8*trainData:size())}}]:mean())

--print '    Average Test Set Error Value top 80%:'
--print(te_crit_error[{{1,math.floor(0.8*testData:size())}}]:mean())

