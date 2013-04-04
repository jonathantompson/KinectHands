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
-- The cuda modules that exist: 
--    github.com/andresy/torch/tree/master/extra/cuda/pkg/cunn

-- Some stuff to look at later (from Soumith)
-- http://code.cogbits.com/wiki/doku.php?id=tutorial_morestuff --> Drop out here
-- http://arxiv.org/pdf/1302.4389v3.pdf --> Max Out
-- http://code.cogbits.com/wiki/doku.php?id=tutorial_unsupervised
-- https://github.com/clementfarabet/torch-tutorials/blob/master/3_unsupervised/A_kmeans.lua
-- https://github.com/clementfarabet/torch-tutorials --> Random tutorials by clement

width = 96
height = 96
num_hpf_banks = 3
dim = width * height
num_coeff = 40
num_coeff_per_feature = 2  -- UV = 2, UVD = 3
frame_stride = 1  -- Only 1 works for now
perform_training = 1
model_filename = 'handmodel.net'
im_dir = "../data/hand_depth_data_processed_for_CN_synthetic/"
test_im_dir = "../data/hand_depth_data_processed_for_CN_test_synthetic/"
test_data_rate = 20  -- this means 1 / 20 FROM THE TRAINING SET will be test data
use_hpf_depth = 1
learning_rate = 1e-3  -- Default 1e-3
l2_reg_param = 1e-4  -- Default 1e-4
learning_rate_decay = 1e-7  -- Default 1e-7
learning_momentum = 0.9 -- Default 0.9 --> Clement suggestion
max_num_epochs = 60
batch_size = 128

-- ********************** Load data from Disk *************************
dofile('load_data.lua')

-- ************ Visualize one of the depth data samples ***************
visualize_data = 0
visualize_data_labels = 1  -- Show features on visualization
dofile('visualize_data.lua')  -- Just define the function
if (visualize_data == 1) then
  VisualizeData(trainData, visualize_data_labels)
  VisualizeData(testData, visualize_data_labels)
end

-- ***************** Define Criterion (loss) function *****************
dofile('define_criterion.lua')

if (perform_training == 1) then

  -- ***************** define the model parameters ********************
  nfeats = 1
  nstates = {{16, 32}, {16, 32}, {16, 32}}
  nstates_nn = 2048
  filtsize = {{7, 6}, {7, 6}, {7, 5}}
  poolsize = {{2, 4}, {2, 2}, {2, 1}}  -- Note: 1 = no pooling
  normkernel = torch.ones(5):float()

  -- *********************** define the model *************************
  dofile('define_model.lua')
  
  -- ************************* Visualize model ************************
  visualize_model = 0
  dofile('visualize_model.lua')

  -- ************************* Enable Logging *************************
  print '==> Creating logs'
  trainLogger = optim.Logger(model_filename .. '.train.log')
  testLogger = optim.Logger(model_filename .. '.test.log')

  -- ************************* Enable Logging *************************
  print '==> Extracting model parameters'
  if model then
     parameters, gradParameters = model:getParameters()
  end

  -- Use SGD
  print '==> Defining optimizer'
  optimState = {
    learningRate = learning_rate,
    weightDecay = l2_reg_param,
    momentum = learning_momentum,
    learningRateDecay = learning_rate_decay
  }
  optimMethod = optim.sgd

  -- ************************* Train function *************************
  dofile('train.lua')

  -- ************************* Test function **************************
  dofile('test.lua')
 
  -- ************************ Saving settings *************************
  dofile('save_settings.lua')

  -- ********************* Perform training loop **********************
  print '==> training!'
  for i = 1,max_num_epochs do
     train()
     test()
  end

else  -- if perform_training
  -- *************** Calculate performance statistics *****************
  dofile('calc_statistics.lua')
end


if false then
  input = {}
  table.insert(input, torch.FloatTensor(1, 96, 96):zero():cuda())
  table.insert(input, torch.FloatTensor(1, 96/2, 96/2):zero():cuda())
  table.insert(input, torch.FloatTensor(1, 96/4, 96/4):zero():cuda())
end






