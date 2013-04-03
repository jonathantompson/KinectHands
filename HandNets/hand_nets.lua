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

torch.setnumthreads(7)
torch.manualSeed(1)

torch.setdefaulttensortype('torch.FloatTensor')
print("GPU That will be used:")
print(cutorch.getDeviceProperties(cutorch.getDevice()))
-- To get GPU Memory usage: nvidia-smi -q -d MEMORY
-- The cuda modules that exist github.com/andresy/torch/tree/master/extra/cuda/pkg/cunn
-- http://code.cogbits.com/wiki/doku.php?id=tutorial_morestuff --> Drop out here
-- http://arxiv.org/pdf/1302.4389v3.pdf --> Max Out
-- https://code.google.com/p/cuda-convnet/
-- http://code.cogbits.com/wiki/doku.php?id=tutorial_unsupervised
-- https://github.com/clementfarabet/torch-tutorials/tree/master/3_unsupervised
-- https://github.com/clementfarabet/torch-tutorials/blob/master/3_unsupervised/A_kmeans.lua <-- Check this one
-- http://arxiv.org/abs/1207.0580

-- Jonathan Tompson
-- NYU, MRL
-- Training script for hand model on kinect depth data

width = 96
height = 96
num_hpf_banks = 3
dim = width * height
num_coeff = 40
frame_stride = 1  -- Only 1 works for now
perform_training = 1
model_filename = 'handmodel.net'
im_dir = "../data/hand_depth_data_processed_for_CN/"
test_im_dir = "../data/hand_depth_data_processed_for_CN_test/"
test_data_rate = 20  -- this means 1 / 20 FROM THE TRAINING SET will be test data
use_hpf_depth = 1
learning_rate = 1e-3  -- Default 1e-3
learning_rate_decay = 1e-2   -- Learning rate = l_0 / (1 + learning_rate_decay * epoch)
l2_reg_param = 5e-4  -- Default 5e-4
max_num_epochs = 60

-- ********************** Load data from Disk *************************
dofile('load_data.lua')

-- ************ Visualize one of the depth data samples ***************
visualize_data = 0
dofile('visualize_data.lua')

-- *********************** Define loss function ***********************
print '==> defining loss function'
-- print '   using ABS criterion'
-- criterion = nn.AbsCriterion()
-- criterion.sizeAverage = false
print '   using MSE criterion'
criterion = nn.MSECriterion()  
criterion:cuda()
print(criterion)

if (perform_training == 1) then

  -- ***************** define the model parameters ********************
  nfeats = 1
  nstates = {{16, 32}, {16, 32}, {16, 32}}
  nstates_nn = 2048
  filtsize = {{5, 6}, {5, 5}, {5, 4}}
  poolsize = {{4, 2}, {2, 2}, {2, 1}}  -- Note: 1 = no pooling
  normkernel = torch.ones(5):float()
  threshold_init_offset = 0.1

  -- *********************** define the model *************************
  dofile('define_model.lua')

  

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
  --[[
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
  --]]
 
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






