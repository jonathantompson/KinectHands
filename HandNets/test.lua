print '    defining test procedure'
-- test function
function test()
  -- local vars
  local time = sys.clock()

  -- averaged param use?
  if average then
    cachedparams = parameters:clone()
    parameters:copy(average)
  end

  local err_ave = 0
  local abs_err_ave = 0

  -- test over test data
  print('==> testing on test set:')
  for t = 1,testData:size() do
    -- disp progress
    progress(t, testData:size())

    -- get new sample
    input = {}
    for j=1,num_hpf_banks-skip_banks do
      table.insert(input, testData.data[j][t])
    end
    for j=1,num_hpf_banks-skip_banks do
      input[j] = input[j]:cuda()
    end
    target = testData.labels[t]

    -- test sample
    pred = model:forward(input)
    cutorch.synchronize()
    if (loss == 0) then
      pred = pred:float()
    end
    err = criterion:forward(pred, target)
  
    err_ave = err_ave + err
    abs_err = err
    if (loss ~= 0) then
      -- abs criterion not supported for cuda yet
      abs_err = abs_criterion:forward(pred:float(), target:float())
    end
    abs_err_ave = abs_err_ave + abs_err
  end

  err_ave = err_ave / testData:size()
  abs_err_ave = abs_err_ave / testData:size()

  -- timing
  time = sys.clock() - time
  time = time / testData:size()
  print("\n==> time to test 1 sample = " .. (time*1000) .. 'ms')

  print("Average loss function value on test set: " .. (err_ave) .. " (using criterion)")
  testLogger:add{['average err'] = string.format('%.8e', abs_err_ave)}
  --testLogger:plot()
  
  -- averaged param use?
  if average then
    -- restore parameters
    parameters:copy(cachedparams)
  end
end
