print '==> loading model from disk:'
model = torch.load(model_filename)

print("==> Measuring test dataset performance:")

te_crit_error = torch.FloatTensor(testData:size())
te_crit_err_by_coeff = torch.zeros(num_coeff):float()
for t=1,testData:size(),1 do
  -- disp progress
  if (math.mod(t, 100) == 1 or t == testData:size()) then
    progress(t, testData:size())
  end
 
  data_pt = {
    input = {},
    target = testData.labels[t]
  }
  for j=1,num_hpf_banks do
    table.insert(data_pt.input, testData.data[j][t])
  end
  for j=1,num_hpf_banks do
    data_pt.input[j] = data_pt.input[j]:cuda()
  end

  pred = model:forward(data_pt.input)
  te_crit_error[t] = math.abs(criterion:forward(pred, data_pt.target))

  err = te_crit_error[t]
  if (err == math.huge or err ~= err) then
    print(string.format("%d, %s is nan or inf!\n", t, testData.files[t]))
  end

  pred = pred:float()
  data_pt.target = data_pt.target:float()
  delta = (pred - data_pt.target)
  delta:abs()
  te_crit_err_by_coeff:add(delta)
end

print '==> Measuring training dataset performance:'

tr_crit_error = torch.FloatTensor(trainData:size())
tr_crit_err_by_coeff  = torch.zeros(num_coeff):float()
for t=1,trainData:size(),1 do
  -- disp progress
  if (math.mod(t, 100) == 1 or t == trainData:size()) then
    progress(t, trainData:size())
  end

  data_pt = {
    input = {},
    target = trainData.labels[t]
  }
  for j=1,num_hpf_banks do
    table.insert(data_pt.input, trainData.data[j][t])
  end
  for j=1,num_hpf_banks do
    data_pt.input[j] = data_pt.input[j]:cuda()
  end

  pred = model:forward(data_pt.input)
  tr_crit_error[t] = math.abs(criterion:forward(pred, data_pt.target))

  err = tr_crit_error[t]
  if (err == math.huge or err ~= err) then
    print(string.format("%d, %s is nan or inf!\n", t, trainData.files[t]))
  end

  pred = pred:float()
  data_pt.target = data_pt.target:float()
  delta = (pred - data_pt.target)
  delta:abs()
  tr_crit_err_by_coeff:add(delta)
end

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

tr_crit_error = torch.sort(tr_crit_error)
te_crit_error = torch.sort(te_crit_error)

print '    Average Training Set Error Value top 20%:'
print(tr_crit_error[{{1,math.floor(0.2*trainData:size())}}]:mean())

print '    Average Test Set Error Value top 20%:'
print(te_crit_error[{{1,math.floor(0.2*testData:size())}}]:mean())

print '    Average Training Set Error Value top 80%:'
print(tr_crit_error[{{1,math.floor(0.8*trainData:size())}}]:mean())

print '    Average Test Set Error Value top 80%:'
print(te_crit_error[{{1,math.floor(0.8*testData:size())}}]:mean())

