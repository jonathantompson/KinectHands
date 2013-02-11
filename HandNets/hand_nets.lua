require 'nn'
require 'image'
require 'torch'
require 'xlua'    -- xlua provides useful tools, like progress bars
require 'optim'   -- an optimization package, for online and batch methods
torch.setnumthreads(4)
-- require 'debugger'

-- Jonathan Tompson
-- NYU, MRL
-- Training script for 26dof hand model coefficient on kinect depth data
-- Model is a 2 stage convnet followed by a 2D neural net
-- Loss function is negative log-likelihood
--  you can run from ide: <dofile 'hand_nets.lua'>

width = 96
height = 96
dim = width * height
frame_skip = 4  -- We don't need every file of the 30fps, so just grab a few
test_data_rate = 5  -- this means 1 / 5 will be test data
num_coeff = 25  -- Keep this at 25!
num_learned_coeff = 25
background_depth = 2000
perform_training = 1
nonlinear = 0  -- 0 = tanh, 1 = SoftShrink
model_filename = 'handmodel'
loss = 0  -- 0 = abs, 1 = mse
fullsize = 1  -- 0 = small, 1 = mid, 2 = big convnet
im_dir = "./hand_depth_data_processed/"
visualize_data = 0
pooling = 2  -- 1,2,.... or math.huge (infinity)

-- ************ Create a filename ***************
if (num_learned_coeff == 25) then
  model_filename = model_filename .. '_fullcoeffs'
end
if (nonlinear == 0) then
  model_filename = model_filename .. '_tanh'
else
  model_filename = model_filename .. '_SoftShrink'
end
if (loss == 0) then
  model_filename = model_filename .. '_abs'
else
  model_filename = model_filename .. '_mse'
end
if (fullsize == 0) then
  model_filename = model_filename .. '_small'
elseif (fullsize == 1) then
  model_filename = model_filename .. '_mid'
else
  model_filename = model_filename .. '_big'
end
if (pooling ~= math.huge) then
  model_filename = model_filename .. string.format("_L%dPooling", pooling)
else
  model_filename = model_filename .. "_LinfPooling"
end

model_filename = model_filename .. '.net'

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
coeffl_files = {}
coeffr_files = {}
depth_files = {}
for i=1,#files,1 do
  if files[i] ~= nil then
    if string.find(files[i], "coeffr_hands_") ~= nil then
      table.insert(coeffr_files, files[i])
    elseif string.find(files[i], "coeffl_hands_") ~= nil then
      table.insert(coeffl_files, files[i])
    elseif string.find(files[i], "hands_") ~= nil then
      table.insert(depth_files, files[i])
    end
  end
end 

-- ************ Load data from Disk ***************
print '==> Loading hand data from directory'
nfiles = math.floor(#depth_files / frame_skip)
tesize = math.floor(nfiles / test_data_rate) + 1
trsize = nfiles - tesize
if tesize <= 0 then
  print("test set size is <= 0")
  return
end
trainData = {
  data = torch.FloatTensor(trsize, 1, height, width),
  labels = torch.DoubleTensor(trsize, num_learned_coeff),
  size = function() return trsize end
}
testData = {
  data = torch.FloatTensor(tesize, 1, height, width),
  labels = torch.DoubleTensor(tesize, num_learned_coeff),
  size = function() return tesize end
}
itr = 1
ite = 1
for i=1,nfiles do
  -- disp progress
  xlua.progress(i, nfiles)

  -- Read in the sample
  coeff_file = torch.DiskFile(im_dir .. coeffr_files[i*frame_skip], 'r')
  coeff_file:binary()
  coeff_data = coeff_file:readFloat(num_coeff)
  coeff_file:close()

  depth_file = torch.DiskFile(im_dir .. depth_files[i*frame_skip], 'r')
  depth_file:binary()
  depth_data = depth_file:readFloat(dim)
  depth_file:close()

  if math.mod(i, test_data_rate) ~= 0 then
    if itr <= trsize then
      -- this sample is training data
      -- We need to convert the short data values to float (and float to double)
      trainData.data[{itr, 1, {}, {}}] = torch.FloatTensor(depth_data, 1, 
        torch.LongStorage{height, width})
      trainData.labels[{itr, {}}] = torch.FloatTensor(coeff_data, 
        num_coeff - num_learned_coeff + 1,
        torch.LongStorage{num_learned_coeff}):double()
      itr = itr + 1
    end
  else
    if ite <= tesize then
      -- this sample is test data
      testData.data[{ite, 1, {}, {}}] = torch.FloatTensor(depth_data, 1, 
        torch.LongStorage{height, width})
      testData.labels[{ite, {}}] = torch.FloatTensor(coeff_data, 
        num_coeff - num_learned_coeff + 1,
        torch.LongStorage{num_learned_coeff}):double()
      ite = ite + 1
    end
  end
end

----- Hack: something goes wrong for certain cases, just redefine what we loaded
tesize = ite - 1
trsize = itr - 1
nfiles = tesize + trsize
trainData.data = trainData.data[{{1,trsize}, {}, {}, {}}]
trainData.labels = trainData.labels[{{1,trsize}, {}}]
trainData.size = function() return trsize end
testData.data = testData.data[{{1,tesize}, {}, {}, {}}]
testData.labels = testData.labels[{{1,tesize}, {}}]
testData.size = function() return tesize end

print(string.format("    Loaded %d test set images and %d training set images", 
  tesize, trsize))

-- ************ Normalize Data ***************
-- EDIT: We don't actually need to normalize now since the data is saved that 
--       way.  This is block commented out.
--[[
print '==> Normalize the depth images'
print '    Normalizing training set'
for i=1,trainData.size(),1 do
  -- disp progress
  xlua.progress(i, trainData:size())

  -- Calculate the mean and std of all hand pixels (not including background)
  sum_pix = 0
  npix = 0
  sum_pix_sq = 0
  for v=1,height,1 do
    for u=1,width,1 do
      pix = trainData.data[{i, 1, v, u}]
      if (pix > 0 and pix < background_depth) then
        sum_pix = sum_pix + pix
        npix = npix + 1
        sum_pix_sq = sum_pix_sq + pix * pix
      end
    end
  end
  mean_pix = sum_pix / npix
  var_pix = (sum_pix_sq / npix) - (mean_pix * mean_pix)
  std_pix = math.sqrt(var_pix)
 
  -- Now subtract off the mean, make all background pixels at 0
  for v=1,height,1 do
    for u=1,width,1 do
      pix = trainData.data[{i, 1, v, u}]
      if (pix > 0 and pix < background_depth) then
        trainData.data[{i, 1, v, u}] = trainData.data[{i, 1, v, u}] - mean_pix
        trainData.data[{i, 1, v, u}] = trainData.data[{i, 1, v, u}] / std_pix
      else
        trainData.data[{i, 1, v, u}] = 0
      end
    end
  end
end

print '    Normalizing test set'
for i=1,testData.size(),1 do
  -- disp progress
  xlua.progress(i, testData:size())

  -- Calculate the mean and std of all hand pixels (not including background)
  sum_pix = 0
  npix = 0
  sum_pix_sq = 0
  for v=1,height,1 do
    for u=1,width,1 do
      pix = testData.data[{i, 1, v, u}]
      if (pix > 0 and pix < background_depth) then
        sum_pix = sum_pix + pix
        npix = npix + 1
        sum_pix_sq = sum_pix_sq + pix * pix
      end
    end
  end
  mean_pix = sum_pix / npix
  var_pix = (sum_pix_sq / npix) - (mean_pix * mean_pix)
  std_pix = math.sqrt(var_pix)
 
  -- Now subtract off the mean, make all background pixels at 0
  for v=1,height,1 do
    for u=1,width,1 do
      pix = testData.data[{i, 1, v, u}]
      if (pix > 0 and pix < background_depth) then
        testData.data[{i, 1, v, u}] = testData.data[{i, 1, v, u}] - mean_pix
        testData.data[{i, 1, v, u}] = testData.data[{i, 1, v, u}] / std_pix
      else
        testData.data[{i, 1, v, u}] = 0
      end
    end
  end
end
--]]

-- ************ Visualize one of the depth data samples ***************
print '==> Visualizing some data samples'
if (visualize_data == 1) then
  n_images = math.min(trainData.size(), 36)
  im = {
    data = trainData.data[{{1,n_images}, {}, {}, {}}]
  }
  im.data = im.data:double()
  image.display{image=im.data, padding=2, zoom=1, scaleeach=false}
  -- image.display(im.data[{1,{},{}}])

  n_images = math.min(testData.size(), 36)
  im = {
    data = testData.data[{{1,n_images}, {}, {}, {}}]
  }
  im.data = im.data:double()
  image.display{image=im.data, padding=2, zoom=1, scaleeach=false}
  -- image.display(im.data[{1,{},{}}])
  im = nil
end

-- ********************** Define loss function **********************
print '==> defining loss function'

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
  noutputs = num_learned_coeff

  -- input dimensions
  nfeats = 1
  ninputs = nfeats*width*height
  if (fullsize == 0) then
    nstates = {8,20,256}
  elseif (fullsize == 1) then
    nstates = {16,64,256}
  else 
    nstates = {16,128,529}
  end
  filtsize = {5, 7}
  poolsize = {2, 4}
  fanin = {4}
  normkernel = image.gaussian1D(7)

  -- ********************** Construct model **********************
  print '==> construct model'
  
  -- a typical convolutional network, with locally-normalized hidden
  -- units, and L2-pooling

  -- Note: the architecture of this convnet is loosely based on Pierre Sermanet's
  -- work on this dataset (http://arxiv.org/abs/1204.3968). In particular
  -- the use of LP-pooling (with P=2) has a very positive impact on
  -- generalization. Normalization is not done exactly as proposed in
  -- the paper, and low-level (first layer) features are not fed to
  -- the classifier.

  model = nn.Sequential()
  tensor_dim = {nfeats, 
                height, 
                width}
  print("Starting Tensor Dimensions:")
  print(tensor_dim)

  -- stage 1 : filter bank -> squashing -> LN pooling -> normalization
  model:add(nn.SpatialConvolutionMap(nn.tables.full(nfeats, nstates[1]), filtsize[1], filtsize[1]))
  if (nonlinear == 1) then 
    model:add(nn.SoftShrink())
  elseif (nonlinear == 0) then
    model:add(nn.Tanh())
  end
  if (pooling ~= math.huge) then
    model:add(nn.SpatialLPPooling(nstates[1], pooling, poolsize[1], poolsize[1], poolsize[1], poolsize[1]))
  else
    model:add(nn.SpatialMaxPooling(poolsize[1], poolsize[1], poolsize[1], poolsize[1]))
  end
  model:add(nn.SpatialSubtractiveNormalization(nstates[1], normkernel))
  tensor_dim = {nstates[1], (tensor_dim[2] - filtsize[1] + 1) / poolsize[1], (tensor_dim[3] - filtsize[1] + 1) / poolsize[1]}
  print("Tensor Dimensions after stage 1:")
  print(tensor_dim)

  -- stage 2 : filter bank -> squashing -> LN pooling -> normalization
  model:add(nn.SpatialConvolutionMap(nn.tables.random(nstates[1], nstates[2], fanin[1]), filtsize[2], filtsize[2]))
  if (nonlinear == 1) then 
    model:add(nn.SoftShrink())
  elseif (nonlinear == 0) then
    model:add(nn.Tanh())
  end
  if (pooling ~= math.huge) then
    model:add(nn.SpatialLPPooling(nstates[2], pooling, poolsize[2], poolsize[2], poolsize[2], poolsize[2]))
  else
    model:add(nn.SpatialMaxPooling(poolsize[2], poolsize[2], poolsize[2], poolsize[2]))
  end
  model:add(nn.SpatialSubtractiveNormalization(nstates[2], normkernel))
  tensor_dim = {nstates[2], (tensor_dim[2] - filtsize[2] + 1) / poolsize[2], (tensor_dim[3] - filtsize[2] + 1) / poolsize[2]}
  print("Tensor Dimensions after stage 2:")
  print(tensor_dim)

  -- stage 3 : standard 2-layer neural network
  vec_length = tensor_dim[1] * tensor_dim[2] * tensor_dim[3]
  model:add(nn.Reshape(vec_length))

  print("Neural net first stage input size")
  print(vec_length);

  model:add(nn.Linear(vec_length, nstates[3]))
  if (nonlinear == 1) then 
    model:add(nn.SoftShrink())
  elseif (nonlinear == 0) then
    model:add(nn.Tanh())
  end

  print("Neural net first stage output size")
  print(nstates[3]);

  model:add(nn.Linear(nstates[3], noutputs))

  print("Final output size")
  print(noutputs)

  -- ********************** Print model **********************
  print '==> here is the model:'
  print(model)

  -- ********************** Visualize model **********************
  -- Visualization is quite easy, using image.display(). Check out:
  -- help(image.display), for more info about options.
  print '==> visualizing ConvNet filters'
  -- image.display{image=model:get(1).weight, padding=2, zoom=4, legend='filters @ layer 1'}
  -- image.display{image=model:get(4).weight, padding=2, zoom=4, nrow=32, legend='filters @ layer 2'}

  -- ********************** Train function **********************
  print '==> defining some tools'
  -- This matrix records the current confusion across classes

  -- Log results to files
  trainLogger = optim.Logger(model_filename .. '.train.log')
  testLogger = optim.Logger(model_filename .. '.test.log')

  -- Retrieve parameters and gradients:
  -- this extracts and flattens all the trainable parameters of the mode
  -- into a 1-dim vector
  if model then
     parameters,gradParameters = model:getParameters()
  end

  -- Use SGD
  batchSize = 1
  optimState = {
    -- Update: parameters = parameters - learningRate * parameters_gradient
    learningRate = 1e-3,
    weightDecay = 0,
    momentum = 0,
    -- current_learning_rate =learningRate / (1 + iteration * learningRateDecay)
    learningRateDecay = 5e-7
  }
  optimMethod = optim.sgd
 
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
    print("==> online epoch # " .. epoch .. ' [batchSize = ' .. batchSize .. ']')
    local ave_err = 0
    local nsamples = 0
    for t = 1,trainData:size(),batchSize do
      -- disp progress
      xlua.progress(t, trainData:size())

      -- create mini batch
      local inputs = {}
      local targets = {}
      for i = t,math.min(t + batchSize-1, trainData:size()) do
         -- load new sample
         local input = trainData.data[shuffle[i]]
         local target = trainData.labels[shuffle[i]]
         input = input:double()
         target = target:double()
         table.insert(inputs, input)
         table.insert(targets, target)
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
        -- local f = 0
        cur_f = 0

        -- evaluate function for complete mini batch
        for i = 1,#inputs do
          -- estimate f
          local output = model:forward(inputs[i])
          local err = criterion:forward(output, targets[i])
          cur_f = cur_f + err
          ave_err = ave_err + err
          nsamples = nsamples + 1

          -- estimate df/dW
          local df_do = criterion:backward(output, targets[i])
          model:backward(inputs[i], df_do)
        end

        -- normalize gradients and f(X)
        gradParameters = gradParameters:div(#inputs)
        cur_f = cur_f/#inputs

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
    print("current loss function value: " .. (ave_err) .. " (using criterion)")
    trainLogger:add{['average err'] = string.format('%.8e', ave_err)}
    --trainLogger:plot()

    -- save/log current net
    os.execute('mkdir -p ' .. sys.dirname(model_filename))
    print('==> saving model to '..model_filename)
    torch.save(model_filename, model)

    -- next epoch
    epoch = epoch + 1
  end

  -- ********************** Test function **********************
  print '==> defining test procedure'
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

    -- test over test data
    print('==> testing on test set:')
    for t = 1,testData:size() do
      -- disp progress
      xlua.progress(t, testData:size())

      -- get new sample
      input = testData.data[t]
      input = input:double()
      target = testData.labels[t]
      target = target:double()

      -- test sample
      pred = model:forward(input)
      err = criterion:forward(pred, target)
  
      err_ave = err_ave + err
    end

    err_ave = err_ave / testData:size()

    -- timing
    time = sys.clock() - time
    time = time / testData:size()
    print("\n==> time to test 1 sample = " .. (time*1000) .. 'ms')

    print("Average loss function value on test set: " .. (err_ave) .. " (using criterion)")
    testLogger:add{['average err'] = string.format('%.8e', err_ave)}
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
  while true do
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
    xlua.progress(t, testData:size())
    -- print(string.format('%d of %d', t, testData:size()))
    -- get new sample
    data_pt = {
      input = testData.data[t],
      target = testData.labels[t]
    }

    data_pt.input = data_pt.input:double()
    data_pt.target = data_pt.target:double()

    -- image.display(data_pt.input)

    pred = model:forward(data_pt.input)

    te_abs_crit_error[t] = math.abs(abs_criterion:forward(pred, data_pt.target))
    te_mse_crit_error[t] = math.abs(mse_criterion:forward(pred, data_pt.target))

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
    xlua.progress(t, trainData:size())
    -- print(string.format('%d of %d', t, trainData:size()))
    -- get new sample
    data_pt = {
      input = trainData.data[t],
      target = trainData.labels[t]
    }

    data_pt.input = data_pt.input:double()
    data_pt.target = data_pt.target:double()

    -- image.display(data_pt.input)

    pred = model:forward(data_pt.input)

    tr_abs_crit_error[t] = math.abs(abs_criterion:forward(pred, data_pt.target))
    tr_mse_crit_error[t] = math.abs(mse_criterion:forward(pred, data_pt.target))

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