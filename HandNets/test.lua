print '==> defining test procedure'
-- test function
function test()
  -- local vars
  local time = sys.clock()

  local err_ave = 0

  -- test over test data
  print('==> testing on test set:')
  for t = 1,testData:size() do
    -- disp progress
    if (math.mod(t, 100) == 1 or t == testData:size()) then
      progress(t, testData:size())
    end

    -- get new sample
    local input = {}
    for j=1,num_hpf_banks do
      table.insert(input, testData.data[j][t])
    end
    for j=1,num_hpf_banks do
      input[j] = input[j]:cuda()
    end
    local target = testData.labels[t]  -- Already cuda

    -- test sample
    local pred = model:forward(input)
    cutorch.synchronize()
    local err = criterion:forward(pred, target)
  
    err_ave = err_ave + err
  end

  err_ave = err_ave / testData:size()

  -- timing
  time = sys.clock() - time
  time = time / testData:size()
  print("\n==> time to test 1 sample = " .. (time*1000) .. 'ms')

  print("Average loss function value on test set: " .. (err_ave) .. " (using criterion)")
  testLogger:add{['average err'] = string.format('%.8e', err_ave)}
end
