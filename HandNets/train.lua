print '==> defining training procedure'
function train(data)

  -- epoch tracker
  epoch = epoch or 1

  -- local vars
  local time = sys.clock()
  local total_t_minibatch = 0
  local total_t_model_forward = 0
  local total_t_criterion_forward = 0
  local total_t_model_backward = 0
  local total_t_criterion_backward = 0

  -- shuffle at each epoch
  local shuffle = torch.randperm(trsize)

  -- do one epoch
  print('==> doing epoch on training data:')
  print("==> online epoch # " .. epoch .. ' [batchSize = ' .. batch_size .. ']')
  local ave_err = 0
  local nsamples = 0
  local next_progress = 50
  for t = 1,data:size(),batch_size do
    -- disp progress
    if (t >= next_progress or t == data:size()) then
      progress(t, data:size())
      next_progress = next_progress + 50
    end

    -- create mini batch
    local t_minibatch = sys.clock()
    local cur_batch_start = t
    local cur_batch_end = math.min(t + batch_size - 1, data:size())
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
      local cur_i = shuffle[i]
      for j=1,num_hpf_banks do
        batchData.data[j][{out_i,{},{},{}}] = data.data[j][{cur_i,{},{},{}}]
      end
      batchData.labels[{out_i,{}}] = data.labels[cur_i]:clone():cuda()
      out_i = out_i + 1
    end
    for j=1,num_hpf_banks do
      batchData.data[j] = batchData.data[j]:cuda()
    end
    t_minibatch = sys.clock() - t_minibatch
    total_t_minibatch = total_t_minibatch + t_minibatch

    -- Visualize data for debug
    -- VisualizeData(batchData, 1)

    -- create closure to evaluate f(X) and df/dX
    local feval = function(x)
      -- get new parameters
      if x ~= parameters then
        parameters:copy(x)
      end

      -- reset gradients
      gradParameters:zero()

      -- f is the average of all criterions
      local f = torch.FloatTensor(cur_batch_size)

      -- evaluate function for complete mini batch
      -- estimate f
      local t_model_forward = sys.clock()
      local output = model:forward(batchData.data)
      t_model_forward = sys.clock() - t_model_forward
      total_t_model_forward = total_t_model_forward + t_model_forward
    
      local t_criterion_forward = sys.clock()
      local err = criterion:forward(output, batchData.labels)
      t_criterion_forward = sys.clock() - t_criterion_forward
      total_t_criterion_forward = total_t_criterion_forward + 
        t_criterion_forward

      f = f + err
      ave_err = ave_err + err
      nsamples = nsamples + 1

      -- estimate df/dW
      local t_criterion_backward = sys.clock()
      local df_do = criterion:backward(output, batchData.labels)
      t_criterion_backward = sys.clock() - t_criterion_backward
      total_t_criterion_backward = total_t_criterion_backward + 
        t_criterion_backward

      local t_model_backward = sys.clock()
      model:backward(batchData.data, df_do)
      t_model_backward = sys.clock() - t_model_backward
      total_t_model_backward = total_t_model_backward + t_model_backward

      -- normalize gradients and f(X)
      -- gradParameters = gradParameters:div(cur_batch_size)
      -- f = f / cur_batch_size  -- TO DO: DOES THIS NEED TO BE NORMALIZED (IT DOESN'T LOOK LIKE IT)

      -- return f and df/dX
      return f, gradParameters
    end

    -- optimize on current mini-batch
    optimMethod(feval, parameters, optimState)
  end

  -- Finish the progress bar
  progress(data:size(), data:size())

  -- time taken
  time = sys.clock() - time
  time = time / data:size()
  print("\n==> time to learn 1 sample = " .. (time*1000) .. 'ms')
  print("==> total time spent creating minibatch = " ..
    total_t_minibatch .. 's')
  print("==> total time spent in forward model = " ..
    total_t_model_forward .. 's')
  print("==> total time spent in forward criterion = " ..
    total_t_criterion_forward .. 's')
  print("==> total time spent in backward model = " ..
    total_t_model_backward .. 's')
  print("==> total time spent in backward criterion = " ..
    total_t_criterion_backward .. 's')

  ave_err = ave_err / nsamples
  print("current loss function value: " .. (ave_err) .. " (using criterion)")
  trainLogger:add{['average err'] = string.format('%.8e', ave_err)}

  -- save/log current net
  os.execute('mkdir -p ' .. sys.dirname(model_filename))
  print('==> saving model to ' .. model_filename)
  torch.save(model_filename, model)

  -- next epoch
  epoch = epoch + 1
end
