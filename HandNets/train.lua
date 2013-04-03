print '    defining training procedure'
function train()

  -- epoch tracker
  epoch = epoch or 1
  cur_learning_rate = learning_rate / (1 + learning_rate_decay * (epoch-1))
  l2_reg_scale = 1 - l2_reg_param * cur_learning_rate

  -- local vars
  local time = sys.clock()
  
  -- shuffle at each epoch
  shuffle = torch.randperm(trsize)

  -- do one epoch
  print('==> doing epoch on training data:')
  print("==> online epoch # " .. epoch)
  print("==> Learning rate " .. cur_learning_rate)
  print("==> l2_reg_scale " .. l2_reg_scale)
  local ave_err = 0
  local ave_abs_err = 0  -- might be same as ave_err if loss = 0
  local nsamples = 0
  for t = 1,trainData:size() do
    -- disp progress
    progress(t, trainData:size())

    -- Collect the current image into a single array
    cur_i = shuffle[t]
    input = {}
    for j=1,num_hpf_banks-skip_banks do
      table.insert(input, trainData.data[j][cur_i])
    end
    for j=1,num_hpf_banks-skip_banks do
      input[j] = input[j]:cuda()
    end
    target = trainData.labels[cur_i]

    -- evaluate function
    -- estimate f
    output = model:forward(input)
    cutorch.synchronize()
    if (loss == 0) then
      output = output:float()
    end

    err = criterion:forward(output, target)
    ave_err = ave_err + err
    nsamples = nsamples + 1
    abs_err = err
    if (loss ~= 0) then
      -- abs criterion not supported for cuda yet
      abs_err = abs_criterion:forward(output:float(), target:float())
    end
    ave_abs_err = ave_abs_err + abs_err

    -- reset gradients
    model:zeroGradParameters()

    -- estimate df/dW
    df_do = criterion:backward(output, target)
    df_do = df_do:cuda()
    model:backward(input, df_do)

    model:updateParameters(cur_learning_rate)

    -- L2 Regularization
    -- Updating weights here escentially means that the learning rate is slightly lower, it will be
    -- learning_rate' = learning_rate * (1 - l2_reg_param * learning_rate) = 0.999999 * learning_rate
    if (math.abs(l2_reg_param) > 1e-9 and epoch > 1) then
          
      for k = 1,num_hpf_banks-skip_banks do
        -- Weight and bias of 1st stage convolution
        model:get(1):get(k):get(1).weight:mul(l2_reg_scale)
        model:get(1):get(k):get(1).bias:mul(l2_reg_scale)
        -- Weight and bias of 2nd stage convolution
        if (poolsize[k+skip_banks][1] == 1) then  -- no pooling in the first stage
          model:get(1):get(k):get(6).weight:mul(l2_reg_scale)
          model:get(1):get(k):get(6).bias:mul(l2_reg_scale)
        else
          model:get(1):get(k):get(7).weight:mul(l2_reg_scale)
          model:get(1):get(k):get(7).bias:mul(l2_reg_scale)
        end
      end
      -- Weight and bias of 1st stage NN
      model:get(3).weight:mul(l2_reg_scale)
      model:get(3).bias:mul(l2_reg_scale)
      -- Weight and bias of 2nd stage NN
      model:get(5).weight:mul(l2_reg_scale)
      model:get(5).bias:mul(l2_reg_scale)
    end

    -- if _DEBUG_ then pause() end
  end

  -- time taken
  time = sys.clock() - time
  time = time / trainData:size()
  print("\n==> time to learn 1 sample = " .. (time*1000) .. 'ms')
  
  ave_err = ave_err / nsamples
  ave_abs_err = ave_abs_err / nsamples
  print("current loss function value: " .. (ave_err) .. " (using criterion)")
  trainLogger:add{['average err'] = string.format('%.8e', ave_abs_err)}
  --trainLogger:plot()

  -- save/log current net
  os.execute('mkdir -p ' .. sys.dirname(model_filename))
  print('==> saving model to '..model_filename)
  torch.save(model_filename, model)

  -- next epoch
  epoch = epoch + 1
end
