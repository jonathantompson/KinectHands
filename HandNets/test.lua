print '==> defining test procedure'
-- test function
function test()
  -- local vars
  local time = sys.clock()

  local err_ave = 0
  local	total_t_minibatch = 0
  local	total_t_forward = 0
  local	total_t_criterion = 0

  -- test over test data
  print('==> testing on test set:')
  local next_progress = 50
  local n_samples = 0
  local batchData = {
    files = {},
    data = {},
    size = function() return 0 end,
    heat_maps = torch.CudaTensor(batch_size, num_features * heat_map_height * heat_map_width)
  }
  for j=1,num_hpf_banks do
    table.insert(batchData.data, torch.CudaTensor(batch_size, 1, bank_dim[j][1], bank_dim[j][2]))
  end

  for t = 1,testData:size(),batch_size do
    collectgarbage()

    -- disp progress
    if (t >= next_progress or t == testData:size()) then
      progress(t, testData:size())
      next_progress = next_progress + 50
    end

    local t_minibatch = sys.clock() 
    -- create mini batch
    local cur_batch_start = t
    local cur_batch_end = math.min(t + batch_size - 1, testData:size())
    local cur_batch_size = cur_batch_end - cur_batch_start + 1
    batchData.size = function() return cur_batch_size end
    local out_i = 1
    for i = cur_batch_start,cur_batch_end do    
      -- Collect the current image and put it into the data slot
      for j=1,num_hpf_banks do
        batchData.data[j][{out_i,{},{},{}}]:copy(testData.data[j][{i,{},{},{}}])
      end
      batchData.heat_maps[{out_i,{}}]:copy(torch.FloatTensor(
        testData.heat_maps[{i,{},{},{}}], 1, torch.LongStorage{num_features *
        heat_map_height * heat_map_width}))
      out_i = out_i + 1
    end
    t_minibatch = sys.clock() - t_minibatch
    total_t_minibatch = total_t_minibatch + t_minibatch

    -- test sample
    local t_forward = sys.clock()
    local pred = model:forward(batchData.data)
    t_forward = sys.clock() - t_forward
    total_t_forward = total_t_forward + t_forward
    
    cutorch.synchronize()

    local t_criterion = sys.clock()
    local err = criterion:forward(pred, batchData.heat_maps)
    t_criterion = sys.clock() - t_criterion
    total_t_criterion = total_t_criterion + t_criterion  

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

  print("==> total time spent creating minibatch = " .. 
    total_t_minibatch .. 's')
  print("==> total time spent in forward model = " ..
    total_t_forward .. 's')
  print("==> total time spent in forward criterion = " ..
    total_t_criterion .. 's')

  print("Average loss function value on test set: " .. (err_ave) .. " (using criterion)")
  testLogger:add{['average err'] = string.format('%.8e', err_ave)}
end
