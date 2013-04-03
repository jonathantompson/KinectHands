abs_criterion = nn.AbsCriterion()
abs_criterion.sizeAverage = false
mse_criterion = nn.MSECriterion()

print '==> loading model from disk:'
model = torch.load(model_filename)

print '==> here is the model:'
print(model)

print("    Measuring test dataset performance:")

te_abs_crit_error = torch.FloatTensor(testData:size())
te_mse_crit_error = torch.FloatTensor(testData:size())
te_err_by_coeff = torch.zeros(num_coeff):float()
for t=1,testData:size(),1 do
progress(t, testData:size())
  -- print(string.format('%d of %d', t, testData:size()))
  -- get new sample
  data_pt = {
    input = {},
    target = testData.labels[t]
  }
  for j=1,num_hpf_banks-skip_banks do
    table.insert(data_pt.input, testData.data[j][t]:cuda())
  end

  -- image.display(data_pt.input[1])

  pred = model:forward(data_pt.input)
  cutorch.synchronize()
  pred = pred:float()

  delta = (pred - data_pt.target)
  delta:abs()
  te_err_by_coeff:add(delta)

  te_abs_crit_error[t] = math.abs(abs_criterion:forward(pred, data_pt.target))
  te_mse_crit_error[t] = math.abs(mse_criterion:forward(pred, data_pt.target))
  err = te_abs_crit_error[t]
  if (err == math.huge or err ~= err) then
    print(string.format("%d, %s is nan or inf!\n", t, trainData.files[t]))
  end


  -- print 'Label parameters:'
  -- print(data_pt.target)
  -- print 'ConvNet parameters:'
  -- print(pred)
  -- print(string.format('error: mse %f, abs %f', te_mse_crit_error[t], te_abs_crit_error[t]))
end

print '    Measuring training dataset performance:'

tr_abs_crit_error = torch.FloatTensor(trainData:size())
tr_mse_crit_error = torch.FloatTensor(trainData:size())
tr_err_by_coeff = torch.zeros(num_coeff):float()
for t=1,trainData:size(),1 do
  progress(t, trainData:size())
  -- print(string.format('%d of %d', t, trainData:size()))
  -- get new sample
  data_pt = {
    input = {},
    target = trainData.labels[t]
  }
  for j=1,num_hpf_banks-skip_banks do
    table.insert(data_pt.input, trainData.data[j][t]:cuda())
  end

  -- image.display(data_pt.input[1])

  pred = model:forward(data_pt.input)
  cutorch.synchronize()
  pred = pred:float()

  delta = (pred - data_pt.target)
  delta:abs()
  tr_err_by_coeff:add(delta)

  tr_abs_crit_error[t] = math.abs(abs_criterion:forward(pred, data_pt.target))
  tr_mse_crit_error[t] = math.abs(mse_criterion:forward(pred, data_pt.target))
  err = tr_abs_crit_error[t]
  if (err == math.huge or err ~= err) then
    print(string.format("%d, %s is nan or inf!\n", t, trainData.files[t]))
  end

  -- print 'Label parameters:'
  -- print(data_pt.target)
  -- print 'ConvNet parameters:'
  -- print(pred)
  -- print(string.format('error: mse %f, abs %f', tr_mse_crit_error[t], tr_abs_crit_error[t]))
end

tr_err_by_coeff:mul(1 / trainData:size())
te_err_by_coeff:mul(1 / testData:size()) 
print 'Average Test Set abs(Error) Value BY COEFF:'
print(te_err_by_coeff)
print 'Average Training Set abs(Error) Value BY COEFF:'
print(tr_err_by_coeff)

print("\n Overall model results for model " .. model_filename .. ":")

print 'Average Training Set Error Value (abs):'
print(tr_abs_crit_error:mean())
print 'Average Training Set Error Value (mse):'
print(tr_mse_crit_error:mean())

print 'Average Test Set Error Value (abs):'
print(te_abs_crit_error:mean())
print 'Average Test Set Error Value (mse):'
print(te_mse_crit_error:mean())

tr_abs_crit_error = torch.sort(tr_abs_crit_error)
tr_mse_crit_error = torch.sort(tr_mse_crit_error)
te_abs_crit_error = torch.sort(te_abs_crit_error)
te_mse_crit_error = torch.sort(te_mse_crit_error)

print 'Average Training Set Error Value top 20% (abs):'
print(tr_abs_crit_error[{{1,math.floor(0.2*trainData:size())}}]:mean())
print 'Average Training Set Error Value top 20% (mse):'
print(tr_mse_crit_error[{{1,math.floor(0.2*trainData:size())}}]:mean())

print 'Average Test Set Error Value top 20% (abs):'
print(te_abs_crit_error[{{1,math.floor(0.2*testData:size())}}]:mean())
print 'Average Test Set Error Value top 20% (mse):'
print(te_mse_crit_error[{{1,math.floor(0.2*testData:size())}}]:mean())

print 'Average Training Set Error Value top 80% (abs):'
print(tr_abs_crit_error[{{1,math.floor(0.8*trainData:size())}}]:mean())
print 'Average Training Set Error Value top 80% (mse):'
print(tr_mse_crit_error[{{1,math.floor(0.8*trainData:size())}}]:mean())

print 'Average Test Set Error Value top 80% (abs):'
print(te_abs_crit_error[{{1,math.floor(0.8*testData:size())}}]:mean())
print 'Average Test Set Error Value top 80% (mse):'
print(te_mse_crit_error[{{1,math.floor(0.8*testData:size())}}]:mean())

