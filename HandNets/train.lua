print '==> defining training procedure'
function train()

  -- epoch tracker
  epoch = epoch or 1

  -- local vars
  local time = sys.clock()
  
  -- shuffle at each epoch
  shuffle = torch.randperm(trsize)

  -- do one epoch
  print('==> doing epoch on training data:')
  print("==> online epoch # " .. epoch .. ' [batchSize = ' .. batch_size .. ']')
  local ave_err = 0
  local nsamples = 0
  local next_progress = 50
  for t = 1,trainData:size(),batch_size do
    -- disp progress
    if (t >= next_progress or t == trainData:size()) then
      progress(t, trainData:size())
      next_progress = next_progress + 50
    end

    -- create mini batch
    local cur_batch_start = t
    local cur_batch_end = math.min(t + batch_size - 1, trainData:size())
    local cur_batch_size = cur_batch_end - cur_batch_start
    local input = {}
    for j=1,num_hpf_banks do
      table.insert(input, torch.FloatTensor(cur_batch_size, 1, bank_dim[j][1], bank_dim[j][2]))
    end
    target = torch.FloatTensor(cur_batch_size, num_coeff)

 -- ************** TO DO: FIX THIS --> IT IS NOW BROKEN **************

    for i = cur_batch_start,cur_batch_end do    
      -- Collect the current image into a single array
      local cur_i = shuffle[i]
      for j=1,num_hpf_banks do
        input[j][{cur_i,{},{},{}}] = trainData.data[j][{cur_i,{},{},{}}]
      end
      for j=1,num_hpf_banks do
        cur_input[j] = cur_input[j]:cuda()
      end
      local cur_target = trainData.labels[cur_i]
      -- Insert the current data into the array
      table.insert(inputs, cur_input)
      table.insert(targets, cur_target)
    end

    for j=1,num_hpf_banks do
      cur_input[j] = cur_input[j]:cuda()
    end

    -- create closure to evaluate f(X) and df/dX
    local feval = function(x)
      -- get new parameters
      if x ~= parameters then
        parameters:copy(x)
      end

      -- reset gradients
      gradParameters:zero()

      -- f is the average of all criterions
      local f = 0

      -- evaluate function for complete mini batch
      for i = 1,#inputs do
        -- estimate f
        local output = model:forward(inputs[i])
        local err = criterion:forward(output, targets[i])
        f = f + err
        ave_err = ave_err + err
        nsamples = nsamples + 1

        -- estimate df/dW
        local df_do = criterion:backward(output, targets[i])
        model:backward(inputs[i], df_do)
      end

      -- normalize gradients and f(X)
      gradParameters = gradParameters:div(#inputs)
      f = f / #inputs

      -- return f and df/dX
      return f, gradParameters
    end

    -- optimize on current mini-batch
    optimMethod(feval, parameters, optimState)
  end

  -- time taken
  time = sys.clock() - time
  time = time / trainData:size()
  print("\n==> time to learn 1 sample = " .. (time*1000) .. 'ms')
  
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
