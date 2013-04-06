print '==> defining test procedure'
-- test function
function test()
  -- local vars
  local time = sys.clock()

  local err_ave = 0

  -- test over test data
  print('==> testing on test set:')
  local next_progress = 50
  local n_samples = 0
  for t = 1,testData:size(),batch_size do
    -- disp progress
    if (t >= next_progress or t == testData:size()) then
      progress(t, testData:size())
      next_progress = next_progress + 50
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
      table.insert(batchData.data, torch.FloatTensor(cur_batch_size, 1, bank_dim[j][1], bank_dim[j][2]))
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

    -- test sample
    local pred = model:forward(batchData.data)
    cutorch.synchronize()
    local err = criterion:forward(pred, batchData.labels)
  
    err_ave = err_ave + err
    n_samples = n_samples + 1
  end

  -- Finish the progress bar
  progress(testData:size(), testData:size())

  err_ave = err_ave / n_samples

  -- timing
  time = sys.clock() - time
  time = time / testData:size()
  print("\n==> time to test 1 sample = " .. (time*1000) .. 'ms')

  print("Average loss function value on test set: " .. (err_ave) .. " (using criterion)")
  testLogger:add{['average err'] = string.format('%.8e', err_ave)}
end
