require 'nn'
require 'cunn'
require 'image'
-- require 'torch'
require 'cutorch'
-- require 'xlua'    -- xlua provides useful tools, like progress bars
require 'optim'   -- an optimization package, for online and batch methods
require 'sys'
-- require 'debugger'

dofile("pbar.lua")
dofile("shuffle_files.lua")
dofile("modules_cc.lua")

torch.setnumthreads(6)
torch.manualSeed(1)

torch.setdefaulttensortype('torch.FloatTensor')
print("GPU That will be used:")
print(cutorch.getDeviceProperties(cutorch.getDevice()))
-- To get GPU Memory usage: nvidia-smi -q -d MEMORY

-- nn.SpatialConvolutionMM = nn.SpatialConvolution

-- Jonathan Tompson
-- NYU, MRL
-- Training script for hand model on kinect depth data

width = 96
height = 96
num_hpf_banks = 3
dim = width * height
num_coeff = 58
perform_training = 1
nonlinear = 0  -- 0 = tanh, 1 = SoftShrink, 2 = ramp
model_filename = 'handmodel2.net'  -- Will also change log and settings file names
loss = 0  -- 0 = abs, 1 = mse
im_dir = "../data/hand_depth_data_processed_for_CN/"
test_im_dir = "../data/hand_depth_data_processed_for_CN_test/"
test_data_rate = 20  -- this means 1 / 20 FROM THE TRAINING SET will be test data
visualize_data = 0
pooling = 2  -- 1,2,.... or math.huge (infinity)
use_hpf_depth = 0
learning_rate = 1e-3  -- Default 1e-3
learning_rate_decay = 5e-7
l2_reg_param = 0
max_num_epochs = 100

-- ******* Some preliminary calculations *********
w = width
h = height
bank_dim = {}
data_file_size = 0
for i=1,num_hpf_banks do
  table.insert(bank_dim, {h, w})
  data_file_size = data_file_size + w * h
  w = w / 2
  h = h / 2
end
w = nil  -- To avoid confusion
h = nil

-- ************ Get Data locations ***************
print '==> Scanning directory for hand data'
if paths.dirp(im_dir) == false then
  print("Couldn't find image directory")
  return
else 
  -- Collect the filenames
  files = {}
  i = 1
  for f in paths.files(im_dir) do
    files[i] = f
    i = i+1
  end
  -- The files are in random order so sort them
  table.sort(files, 
    function (a, b) 
      return string.lower(a) < string.lower(b) 
    end)
end
-- Partition files into their respective groups
-- coeffl_files = {}
coeffr_files = {}
hpf_depth_files = {}
depth_files = {}
for i=1,#files,1 do
  if files[i] ~= nil then
    if string.find(files[i], "coeffr_hands_") ~= nil then
      table.insert(coeffr_files, files[i])
--    elseif string.find(files[i], "coeffl_hands_") ~= nil then
--      table.insert(coeffl_files, files[i])
    elseif string.find(files[i], "hpf_hands_") ~= nil then
      table.insert(hpf_depth_files, files[i])
    elseif string.find(files[i], "processed_hands_") ~= nil then
      table.insert(depth_files, files[i])
    end
  end
end 

if paths.dirp(test_im_dir) == false then
  print("Couldn't find test image directory")
  return
else 
  -- Collect the filenames
  files = {}
  i = 1
  for f in paths.files(test_im_dir) do
    files[i] = f
    i = i+1
  end
  -- The files are in random order so sort them
  table.sort(files, 
    function (a, b) 
      return string.lower(a) < string.lower(b) 
    end)
end
-- Partition files into their respective groups
-- coeffl_files = {}
test_coeffr_files = {}
test_hpf_depth_files = {}
test_depth_files = {}
for i=1,#files,1 do
  if files[i] ~= nil then
    if string.find(files[i], "coeffr_hands_") ~= nil then
      table.insert(test_coeffr_files, files[i])
--    elseif string.find(files[i], "coeffl_hands_") ~= nil then
--      table.insert(coeffl_files, files[i])
    elseif string.find(files[i], "hpf_hands_") ~= nil then
      table.insert(test_hpf_depth_files, files[i])
    elseif string.find(files[i], "processed_hands_") ~= nil then
      table.insert(test_depth_files, files[i])
    end
  end
end 

if (use_hpf_depth == 1) then
  im_files = hpf_depth_files
  test_im_files = test_hpf_depth_files
else
  im_files = depth_files
  test_im_files = test_depth_files
end

-- ************ Randomly permute the files ***********
shuffle_files_right(coeffr_files, im_files)
shuffle_files_right(test_coeffr_files, test_im_files)

-- ************ Load data from Disk ***************
print '==> Loading hand data from directory'
nfiles = #im_files
tesize = math.floor(nfiles / test_data_rate) + 1
trsize = nfiles - tesize
tesize = tesize + #test_im_files  -- Also add in the training files from the separate dir
if tesize <= 0 then
  print("test set size is <= 0")
  return
end
trainData = {
  files = {},
  data = {},  -- Multi-bank input (usually 3)
  labels = torch.FloatTensor(trsize, num_coeff),
  size = function() return trsize end
}
testData = {
  files = {},
  data = {},
  labels = torch.FloatTensor(tesize, num_coeff),
  size = function() return tesize end
}

-- LOAD IN TRAINING SET DIRECTORY --> SOME IMAGES GO TO TEST SET AS WELL!
itr = 1
ite = 1
for i=1,nfiles do
  -- disp progress
  progress(i, nfiles)

  -- Read in the sample
  coeff_file = torch.DiskFile(im_dir .. coeffr_files[i], 'r')
  coeff_file:binary()
  coeff_data = coeff_file:readFloat(num_coeff)
  coeff_file:close()

  hpf_depth_file = torch.DiskFile(im_dir .. im_files[i],'r')
  hpf_depth_file:binary()
  hpf_depth_data = hpf_depth_file:readFloat(data_file_size)
  hpf_depth_file:close()

  if math.mod(i, test_data_rate) ~= 0 then
    if itr <= trsize then
      -- this sample is training data
      -- We need to split the long vector
      ind = 1
      cur_sample = {}
      for j=1,num_hpf_banks do
        cur_bank = torch.FloatTensor(hpf_depth_data, ind, 
          torch.LongStorage{1, bank_dim[j][1], bank_dim[j][2]}):float()
        ind = ind + (bank_dim[j][1]*bank_dim[j][2]) -- Move pointer forward
        table.insert(cur_sample, cur_bank)
      end
      table.insert(trainData.data, cur_sample)
      trainData.labels[{itr, {}}] = torch.FloatTensor(coeff_data, 1,
        torch.LongStorage{num_coeff}):float()
      trainData.files[itr] = im_files[i]
      itr = itr + 1
    end
  else
    if ite <= tesize then
      -- this sample is test data
      -- We need to split the long vector
      ind = 1
      cur_sample = {}
      for j=1,num_hpf_banks do
        cur_bank = torch.FloatTensor(hpf_depth_data, ind, 
          torch.LongStorage{1, bank_dim[j][1], bank_dim[j][2]})
        ind = ind + (bank_dim[j][1]*bank_dim[j][2]) -- Move pointer forward
        table.insert(cur_sample, cur_bank)
      end
      table.insert(testData.data, cur_sample)
      testData.labels[{ite, {}}] = torch.FloatTensor(coeff_data, 1,
        torch.LongStorage{num_coeff}):float()
      testData.files[ite] = im_files[i]
      ite = ite + 1
    end
  end
end

-- NOW LOAD IN TEST SET DIRECTORY
for i=1,#test_im_files do
  -- disp progress
  progress(i, #test_im_files)

  -- Read in the sample
  coeff_file = torch.DiskFile(test_im_dir .. test_coeffr_files[i], 'r')
  coeff_file:binary()
  coeff_data = coeff_file:readFloat(num_coeff)
  coeff_file:close()

  hpf_depth_file = torch.DiskFile(im_dir .. im_files[i],'r')
  hpf_depth_file:binary()
  hpf_depth_data = hpf_depth_file:readFloat(data_file_size)
  hpf_depth_file:close()
  if ite <= tesize then
    -- this sample is test data
    -- We need to split the long vector
    ind = 1
    cur_sample = {}
    for j=1,num_hpf_banks do
      cur_bank = torch.FloatTensor(hpf_depth_data, ind, 
        torch.LongStorage{1, bank_dim[j][1], bank_dim[j][2]})
      ind = ind + (bank_dim[j][1]*bank_dim[j][2]) -- Move pointer forward
      table.insert(cur_sample, cur_bank)
    end
    table.insert(testData.data, cur_sample)
    testData.labels[{ite, {}}] = torch.FloatTensor(coeff_data, 1,
      torch.LongStorage{num_coeff}):float()
    testData.files[ite] = im_files[i]
    ite = ite + 1
  end
end

----- Hack: something goes wrong for certain cases, just redefine what we loaded
tesize = ite - 1
trsize = itr - 1
nfiles = tesize + trsize
for i=trsize+1,#trainData.files do
  table.remove(trainData.files, trsize+1)
  table.remove(trainData.data, trsize+1)
end
trainData.labels = trainData.labels[{{1,trsize}, {}}]
trainData.size = function() return trsize end
for i=tesize+1,#testData.files do
  table.remove(testData.files, tesize+1)
  table.remove(testData.data, tesize+1)
end
testData.labels = testData.labels[{{1,tesize}, {}}]
testData.size = function() return tesize end

print(string.format("    Loaded %d test set images and %d training set images", 
  tesize, trsize))

-- ************ Visualize one of the depth data samples ***************
print '==> Visualizing some data samples'
if (visualize_data == 1) then
  n_images = math.min(trainData.size(), 256)
  for j=1,num_hpf_banks do
    im = {
      data = torch.FloatTensor(n_images, bank_dim[j][1], bank_dim[j][2])
    }
    for k=1,n_images do
      im.data[{{k},{},{}}] = trainData.data[k][j]
    end
    im.data = im.data:double()
    image.display{image=im.data, padding=2, nrow=math.floor(math.sqrt(n_images)), zoom=(0.75*math.pow(2,j-1)), scaleeach=false}
  end

  n_images = math.min(testData.size(), 256)
  for j=1,num_hpf_banks do
    im = {
      data = torch.FloatTensor(n_images, bank_dim[j][1], bank_dim[j][2])
    }
    for k=1,n_images do
      im.data[{{k},{},{}}] = testData.data[k][j]
    end
    im.data = im.data:double()
    image.display{image=im.data, padding=2, nrow=math.floor(math.sqrt(n_images)), zoom=(0.75*math.pow(2,j-1)), scaleeach=false}
  end

  im = nil
end

-- ********************** Converting data to cuda **********************
print '==> Converting data to cudaTensor'
for j=1,testData.size() do
  for k=1,num_hpf_banks do
    testData.data[j][k] = testData.data[j][k]:cuda()
  end
end
for j=1,trainData.size() do
  for k=1,num_hpf_banks do
    trainData.data[j][k] = trainData.data[j][k]:cuda()
  end
end

-- ********************** Define loss function **********************
print '==> defining loss function'

abs_criterion = nn.AbsCriterion()
abs_criterion.sizeAverage = false

if (loss == 0) then
  -- Absoulte Value criterion: loss(x,y) = 1/n \sum |x_i-y_i|.
  print '   using ABS criterion'
  criterion = nn.AbsCriterion()
  criterion.sizeAverage = false
elseif (loss == 1) then
  print '   using MSE criterion'
  criterion = nn.MSECriterion()  
else
  print("loss should be 0 or 1")
  return
end

print(criterion)

if (perform_training == 1) then

  -- ************ define the model parameters ***************
  -- output dimensions
  noutputs = num_coeff

  -- input dimensions
  nfeats = 1
  nstates = {{8, 32}, {8, 32}, {8, 32}}
  nstates_nn = 1024
  filtsize = {{5, 7}, {5, 5}, {5, 5}}
  poolsize = {{2, 4}, {2, 2}, {1, 2}}  -- Note: 1 = no pooling
  fanin = {{4}, {4}, {4}}
  normkernel = image.gaussian1D(7)

  -- ********************** Construct model **********************
  print '==> construct model'

  model = nn.Sequential()  -- top level model

  -- define the seperate convnet banks (that will execute in parallel)
  banks = {}
  banks_total_output_size = 0
  for j=1,num_hpf_banks do
    table.insert(banks, nn.Sequential():cuda())
    tensor_dim = {1, bank_dim[j][1], bank_dim[j][2]}

    -- stage 1 : filter bank -> squashing -> LN pooling -> normalization
    -- *********** TO DO: SpatialConvolutionMap IS NOT IMPLEMENTED IN CUDA YET **************
    -- banks[j]:add(nn.SpatialConvolutionMap(nn.tables.full(nfeats, nstates[j][1]), filtsize[j][1], filtsize[j][1]))
    banks[j]:add(nn.SpatialConvolution(nfeats, nstates[j][1], filtsize[j][1], filtsize[j][1]):cuda())

    if (nonlinear == 1) then 
      banks[j]:add(nn.SoftShrink():cuda())
    elseif (nonlinear == 0) then
      banks[j]:add(nn.Tanh():cuda())
    elseif (nonlinear == 2) then
      banks[j]:add(nn.ramp():cuda())
    end

    if (poolsize[j][1] > 1) then
      if (pooling ~= math.huge) then
        banks[j]:add(nn.SpatialLPPooling(nstates[j][1], pooling, poolsize[j][1], poolsize[j][1], poolsize[j][1], poolsize[j][1]):cuda())
      else
        banks[j]:add(nn.SpatialMaxPooling(poolsize[j][1], poolsize[j][1], poolsize[j][1], poolsize[j][1]):cuda())
      end
    end

    -- *********** TO DO: SpatialSubtractiveNormalization IS NOT IMPLEMENTED IN CUDA YET **************
    banks[j]:add(nn.Copy('torch.CudaTensor', 'torch.FloatTensor'))
    banks[j]:add(nn.SpatialSubtractiveNormalization(nstates[j][1], normkernel))
    banks[j]:add(nn.Copy('torch.FloatTensor', 'torch.CudaTensor'))

    tensor_dim = {nstates[j][1], (tensor_dim[2] - filtsize[j][1] + 1) / 
      poolsize[j][1], (tensor_dim[3] - filtsize[j][1] + 1) / poolsize[j][1]}
    print(string.format("Tensor Dimensions after stage 1 bank %d:", j))
    print(tensor_dim)

    -- stage 2 : filter bank -> squashing -> LN pooling -> normalization
    -- *********** TO DO: SPATIALCONVOLUTIONMAP IS NOT IMPLEMENTED IN CUDA YET **************
    -- banks[j]:add(nn.SpatialConvolutionMap(nn.tables.random(nstates[j][1], nstates[j][2], fanin[j][1]), filtsize[j][2], filtsize[j][2]))
    banks[j]:add(nn.SpatialConvolution(nstates[j][1], nstates[j][2], filtsize[j][2], filtsize[j][2]):cuda())
    if (nonlinear == 1) then 
      banks[j]:add(nn.SoftShrink():cuda())
    elseif (nonlinear == 2) then
      banks[j]:add(nn.ramp():cuda())
    elseif (nonlinear == 0) then
      banks[j]:add(nn.Tanh():cuda())
    end
    if (poolsize[j][2] > 1) then
      if (pooling ~= math.huge) then
        banks[j]:add(nn.SpatialLPPooling(nstates[j][2], pooling, 
          poolsize[j][2], poolsize[j][2], poolsize[j][2], poolsize[j][2]):cuda())
      else
        banks[j]:add(nn.SpatialMaxPooling(poolsize[j][2], poolsize[j][2], 
          poolsize[j][2], poolsize[j][2]):cuda())
      end
    end

    -- *********** TO DO: SpatialSubtractiveNormalization IS NOT IMPLEMENTED IN CUDA YET **************
    banks[j]:add(nn.Copy('torch.CudaTensor', 'torch.FloatTensor'))
    banks[j]:add(nn.SpatialSubtractiveNormalization(nstates[j][2], normkernel))
    banks[j]:add(nn.Copy('torch.FloatTensor', 'torch.CudaTensor'))

    tensor_dim = {nstates[j][2], (tensor_dim[2] - filtsize[j][2] + 1) / 
      poolsize[j][2], (tensor_dim[3] - filtsize[j][2] + 1) / poolsize[j][2]}
    print(string.format("Tensor Dimensions after stage 2 bank %d:", j))
    print(tensor_dim)

    vec_length = tensor_dim[1] * tensor_dim[2] * tensor_dim[3]
    banks[j]:add(nn.Reshape(vec_length):cuda())
    banks_total_output_size = banks_total_output_size + vec_length

    print(string.format("Bank %d output length:", j))
    print(vec_length);
  end

  -- Now join the banks together!
  -- Parallel applies ith member module to the ith input, and outpus a table
  parallel = nn.ParallelTable():cuda()
  for j=1,num_hpf_banks do
    parallel:add(banks[j])
  end
  model:add(parallel)
  model:add(nn.JoinTable(1):cuda())  -- Take the table of tensors and concat them

  -- stage 3 : standard 2-layer neural network

  print("Neural net first stage input size")
  print(banks_total_output_size);

  model:add(nn.Linear(banks_total_output_size, nstates_nn):cuda())
  if (nonlinear == 1) then 
    model:add(nn.SoftShrink():cuda())
  elseif (nonlinear == 2) then
    model:add(nn.ramp():cuda())
  elseif (nonlinear == 0) then
    model:add(nn.Tanh():cuda())
  end

  print("Neural net first stage output size")
  print(nstates_nn);

  model:add(nn.Linear(nstates_nn, noutputs):cuda())

  print("Final output size")
  print(noutputs)

  -- ********************** Visualize model **********************
  -- print '==> visualizing ConvNet filters'
  -- image.display{image=model:get(1):get(1):get(1).weight, padding=2, zoom=4, legend='filters @ layer 1'}
  -- image.display{image=model:get(1):get(1):get(4).weight, padding=2, zoom=4, nrow=32, legend='filters @ layer 2'}

  -- ********************** Print model **********************
  -- print '==> here is the model:'
  -- print(model)

  -- ********************** Train function **********************
  print '==> defining some tools'
  -- This matrix records the current confusion across classes

  -- Log results to files
  print '    Creating logs'
  trainLogger = optim.Logger(model_filename .. '.train.log')
  testLogger = optim.Logger(model_filename .. '.test.log')

  -- Retrieve parameters and gradients:
  -- this extracts and flattens all the trainable parameters of the mode
  -- into a 1-dim vector
  print '    Extracting model parameters (this may take a while)'
  if model then
     parameters, gradParameters = model:getParameters()
  end

  -- Use SGD
  print '    Defining optimizer'
  optimState = {
    -- Update: parameters = parameters - learningRate * parameters_gradient
    learningRate = learning_rate,
    weightDecay = 0,
    momentum = 0,
    -- current_learning_rate =learningRate / (1 + iteration * learningRateDecay)
    learningRateDecay = learning_rate_decay
  }
  optimMethod = optim.sgd
 
  print '    defining training procedure'

  function train()

    -- epoch tracker
    epoch = epoch or 1

    -- local vars
    local time = sys.clock()
  
    -- shuffle at each epoch
    shuffle = torch.randperm(trsize)

    -- do one epoch
    print('==> doing epoch on training data:')
    print("==> online epoch # " .. epoch)
    local ave_err = 0
    local ave_abs_err = 0  -- might be same as ave_err if loss = 0
    local nsamples = 0
    for t = 1,trainData:size() do
      -- disp progress
      progress(t, trainData:size())

      -- Collect the current image into a single array
      cur_i = shuffle[t]

      -- create closure to evaluate f(X) and df/dX
      local feval = function(x)
        -- get new parameters
        if x ~= parameters then
          parameters:copy(x)
        end

        -- reset gradients
        gradParameters:zero()

        -- f is the average of all criterions
        -- local f = 0
        cur_f = 0

        -- evaluate function
        -- estimate f
        output = model:forward(trainData.data[cur_i])
        cutorch.synchronize()
        output = output:float()

        err = criterion:forward(output, trainData.labels[cur_i])
        cur_f = cur_f + err
        ave_err = ave_err + err
        nsamples = nsamples + 1
        abs_err = err
        if (loss ~= 0) then
          abs_err = abs_criterion:forward(output, trainData.labels[cur_i])
        end
        ave_abs_err = ave_abs_err + abs_err

        -- L2 Regularization
        l2_reg_scale = 1 - l2_reg_param * learning_rate
--        for k = 1,num_hpf_banks do
--          -- Weight and bias of 1st stage convolution
--          model:get(1):get(k):get(1).weight:mul(l2_reg_scale)
--          model:get(1):get(k):get(1).bias:mul(l2_reg_scale)
--          -- Weight and bias of 2nd stage convolution
--          model:get(1):get(k):get(7).weight:mul(l2_reg_scale)
--          model:get(1):get(k):get(7).bias:mul(l2_reg_scale)
--        end
        -- Weight and bias of 1st stage NN
        model:get(3).weight:mul(l2_reg_scale)
        model:get(3).bias:mul(l2_reg_scale)
        -- Weight and bias of 2nd stage NN
        model:get(5).weight:mul(l2_reg_scale)
        model:get(5).bias:mul(l2_reg_scale)

        -- estimate df/dW
        df_do = criterion:backward(output, trainData.labels[cur_i])
        df_do = df_do:cuda()
        model:backward(trainData.data[cur_i], df_do)

        -- normalize gradients and f(X)
        -- gradParameters = gradParameters:div(#inputs)
        -- cur_f = cur_f/#inputs

        -- return f and df/dX
        return cur_f, gradParameters
      end

      -- if _DEBUG_ then pause() end

      -- optimize on current mini-batch
      optimMethod(feval, parameters, optimState)
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

  -- ********************** Test function **********************
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

      -- test sample
      pred = model:forward(testData.data[t])
      cutorch.synchronize()
      pred = pred:float()
      err = criterion:forward(pred, testData.labels[t])
  
      err_ave = err_ave + err
      abs_err = err
      if (loss ~= 0) then
        abs_err = abs_criterion:forward(pred, testData.labels[t])
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

  dofile 'save_settings.lua'

  -- ********************** Training **********************
  print '==> training!'
  for i = 1,max_num_epochs do
     train()
     test()
  end

else  -- if perform_training
  -- ********************** Statistics Run **********************

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
  for t=1,testData:size(),1 do
    progress(t, testData:size())
    -- print(string.format('%d of %d', t, testData:size()))

    pred = model:forward(testData.data[t])
    cutorch.synchronize()
    pred = pred:float()

    te_abs_crit_error[t] = math.abs(abs_criterion:forward(pred, testData.labels[t]))
    te_mse_crit_error[t] = math.abs(mse_criterion:forward(pred, testData.labels[t]))
    err = te_abs_crit_error[t];
    if (err == math.huge or err ~= err) then
      print(string.format("%d, %s is nan or inf!\n", t, trainData.files[t]));
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
  for t=1,trainData:size(),1 do
    progress(t, trainData:size())
    -- print(string.format('%d of %d', t, trainData:size()))

    -- image.display(data_pt.input)

    pred = model:forward(trainData.data[t]):float()

    tr_abs_crit_error[t] = math.abs(abs_criterion:forward(pred, trainData.labels[t]))
    tr_mse_crit_error[t] = math.abs(mse_criterion:forward(pred, trainData.labels[t]))
    err = tr_abs_crit_error[t];
    if (err == math.huge or err ~= err) then
      print(string.format("%d, %s is nan or inf!\n", t, trainData.files[t]));
    end

    -- print 'Label parameters:'
    -- print(data_pt.target)
    -- print 'ConvNet parameters:'
    -- print(pred)
    -- print(string.format('error: mse %f, abs %f', tr_mse_crit_error[t], tr_abs_crit_error[t]))
  end

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

  print 'All values in groups of 5 (for copy and paste)'
  print(string.format('%.10f, %.10f, %.10f, %.10f, %.10f', tr_abs_crit_error:mean(), tr_mse_crit_error:mean(), te_abs_crit_error:mean(), te_mse_crit_error:mean(), tr_abs_crit_error[{{1,math.floor(0.2*trainData:size())}}]:mean()))
  print(string.format('%.10f, %.10f, %.10f, %.10f, %.10f', tr_mse_crit_error[{{1,math.floor(0.2*trainData:size())}}]:mean(), te_abs_crit_error[{{1,math.floor(0.2*testData:size())}}]:mean(), te_mse_crit_error[{{1,math.floor(0.2*testData:size())}}]:mean(), tr_abs_crit_error[{{1,math.floor(0.8*trainData:size())}}]:mean(), tr_mse_crit_error[{{1,math.floor(0.8*trainData:size())}}]:mean()))
  print(string.format('%.10f, %.10f', te_abs_crit_error[{{1,math.floor(0.8*testData:size())}}]:mean(), te_mse_crit_error[{{1,math.floor(0.8*testData:size())}}]:mean()))
end


if false then
  input = {}
  table.insert(input, torch.FloatTensor(1, 96, 96):cuda())
  table.insert(input, torch.FloatTensor(1, 96/2, 96/2):cuda())
  table.insert(input, torch.FloatTensor(1, 96/4, 96/4):cuda())
end






