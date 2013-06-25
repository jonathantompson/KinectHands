-- Jonathan Tompson
-- NYU, MRL
-- Training script for hand model on depth data

require 'nn'
require 'cunn'
require 'image'
require 'torch'
require 'cutorch'
require 'optim'
require 'sys'
require 'xlua'
require 'parallel'
-- require 'debugger'

dofile("pbar.lua")
dofile("shuffle_files.lua")
dofile("modules_cc.lua")

torch.setnumthreads(8)
torch.manualSeed(1)
math.randomseed(1)

torch.setdefaulttensortype('torch.FloatTensor')
print("GPU That will be used:")
print(cutorch.getDeviceProperties(cutorch.getDevice()))
-- To get GPU Memory usage: nvidia-smi -q -d MEMORY
-- The cuda modules that exist: 
--    github.com/andresy/torch/tree/master/extra/cuda/pkg/cunn

-- Some stuff to look at later (from Soumith)
-- http://code.cogbits.com/wiki/doku.php?id=tutorial_morestuff --> Drop out
-- http://arxiv.org/pdf/1302.4389v3.pdf --> Max Out
-- http://code.cogbits.com/wiki/doku.php?id=tutorial_unsupervised
-- https://github.com/clementfarabet/torch-tutorials

width = 96
height = 96
heat_map_width = 18  -- Decimation should equal the convnet pooling
heat_map_height = 18
heat_map_sigma = 0.75  -- Formally 0.75
num_hpf_banks = 3
dim = width * height
num_coeff = 42  -- 8 fingers + 3 thumb + 3 palm positions
num_coeff_per_feature = 3  -- UV = 2, UVD = 3
num_features = num_coeff / num_coeff_per_feature
perform_training = 1
regenerate_heat_maps = 1  -- otherwise it will load them from file
model_filename = 'handmodel.net'
im_dir = "../data/hand_depth_data_processed_for_CN/"
test_im_dir = "../data/hand_depth_data_processed_for_CN_testset/"
heatmap_dir = "../data/heatmaps/"
use_hpf_depth = 1
learning_rate = 2e-0  -- Default 1e-1 (MSE, 1e-4 ABS)
l2_reg_param = 1e-7  -- Default 2e-4
learning_rate_decay = 1e-8  -- Default 1e-6
learning_momentum = 0.9 -- Default 0.9 --> Clement suggestion
max_num_epochs = 500
batch_size = 64  -- Default 128 (BUT MAYBE 32 IS BETTER!)

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

-- ********************** Load data from Disk *************************
dofile('load_data.lua')

-- ************ Visualize one of the depth data samples ***************
visualize_data = 0
dofile('visualize_data.lua')  -- Just define the function:
-- VisualizeData(x, plot_labels, num_banks, n_tiles, zoom_factor)
if (visualize_data == 1) then
  VisualizeData(trainData)
  VisualizeData(testData)
  VisualizeImage(trainData, 1)
  VisualizeImage(testData, 1)
  VisualizeHeatMap(trainData, 1, 4)
  VisualizeHeatMap(testData, 1, 4)
end

-- ***************** Define Criterion (loss) function *****************
dofile('define_criterion.lua')

if (perform_training == 1) then

  -- ***************** define the model parameters ********************
  nfeats = 1
  nstates = {{16, 32}, {16, 32}, {16, 32}}  -- MUST BE MULTIPLES OF 16!
  nn_stg1_out_size = (heat_map_width * heat_map_height * num_features)  -- formally * 2
  filtsize = {{5, 6}, {5, 5}, {4, 4}}
  poolsize = {{4, 2}, {2, 2}, {1, 2}}  -- Note: 1 = no pooling

  -- *********************** define the model *************************
  collectgarbage()
  dofile('define_model.lua')
  
  -- ************************* Visualize model ************************
  visualize_model = 0
  dofile('visualize_model.lua')

  -- ************************* Enable Logging *************************
  print '==> Creating logs'
  trainLogger = optim.Logger(model_filename .. '.train.log')
  testLogger = optim.Logger(model_filename .. '.test.log')
  collectgarbage()

  -- ************************* Enable Logging *************************
  print '==> Extracting model parameters'
  -- This wont work unless you force a garbagecollect() in Module.lua
  -- Replace the file in /usr/local/share/torch/lua/nn/Module.lua with the one
  -- in this directory.
  if model then
     parameters, gradParameters = model:getParameters()
  end
  collectgarbage()

  -- Use SGD
  print '==> Defining optimizer'
  optimState = {
    learningRate = learning_rate,
    weightDecay = l2_reg_param,
    momentum = learning_momentum,
    learningRateDecay = learning_rate_decay  -- clr = lr / (1 + nevals * decay)
  }
  optimMethod = optim.sgd

  -- ************************* Train function *************************
  dofile('train.lua')

  -- ************************* Test function **************************
  dofile('test.lua')

  -- ********************* Database manipulation **********************
  dofile('preturb.lua')
  dofile('preturb_send_recieve.lua')
  -- rotatedData = preturbManual(testData, testData:size())
  -- VisualizeData(rotatedData)
  -- VisualizeData(testData)
  -- VisualizeImage(rotatedData, 1)
  -- VisualizeHeatMap(rotatedData, 1, 4)
  --[[
  ---- EXAMPLE CODE FOR PRETURB SINGLE IMAGE CODE
  dofile("preturb_data_and_labels.lua")
  ind = 1000
  im = testData.data[1][{ind,{1},{},{}}]
  lab = testData.labels[{ind,{}}]
  hm = testData.heat_maps[{ind,{}}]
  image.display(im)
  for i=1,20 do
    im_out, lab_out, hm_out, deg_rot, scale, transv, transu =
      preturbDataAndLabels(im, lab, hm)               
    image.display(im_out)
  end
  --]]
 
  -- ************************ Saving settings *************************
  dofile('save_settings.lua')

  -- ********************* Perform training loop **********************
  print '==> training!'
  test()

  function trainLoop()  
    c = parallel.fork()  -- Spawn a new thread for database manipulation
    c:exec(preturbThread)
    print("trainingThread(): spawned preturbThread(), waiting 5 secs...")
    os.execute("sleep 5")

    print("trainingThread(): Sending database to preturbThread()")
    sendDatabase(c, trainData, num_hpf_banks)
    print("trainingThread(): database sent starting loop...")

    for i = 1,max_num_epochs do
      collectgarbage()
      -- tell the training set generator to create a new set in the background:
      c:join()

      if (i == 1) then
        current_training_data = trainData
      end

      train(current_training_data)
      test()
      
      -- Recieve the new training data from the background thread
      current_training_data, num_hpf_banks = recieveDatabase( c )
    end

    -- Tell the preturb thread that we're all done
    c:join('break')
  end

  -- protected execution:
  ok,err = pcall(trainLoop)
  if not ok then print(err) end
  parallel.close()

--[[
  -- Train in the same thread as rotations (fallback)
  for i = 1,max_num_epochs do
    if ( i == 1 ) then
      current_training_data = trainData
    else
      current_training_data = preturbManual(trainData, trainData:size())
    end
    train(current_training_data)
    test()
  end
--]]

--[[
  -- Simple training loop: No per-epoch rotations
  for i = 1,max_num_epochs do
    train(trainData)
    test()
  end
--]]

else  -- if perform_training
  -- *************** Calculate performance statistics *****************
  dofile('calc_statistics.lua')
end


if false then
  input = {}
  table.insert(input, torch.FloatTensor(1, 96, 96):zero():cuda())
  table.insert(input, torch.FloatTensor(1, 96/2, 96/2):zero():cuda())
  table.insert(input, torch.FloatTensor(1, 96/4, 96/4):zero():cuda())

  batch_input = {}
  table.insert(batch_input, torch.rand(128, 4, 96, 96):cuda())
  table.insert(batch_input, torch.rand(128, 4, 96/2, 96/2):cuda())
  table.insert(batch_input, torch.rand(128, 4, 96/4, 96/4):cuda())
end






