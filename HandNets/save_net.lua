require 'nn'
require 'image'
require 'torch'
require 'xlua'    -- xlua provides useful tools, like progress bars
require 'optim'   -- an optimization package, for online and batch methods
torch. setnumthreads(4)
dofile("saveConvStage.lua")  -- Load in helper function
dofile("saveNNStage.lua")  -- Load in helper function
-- require 'debugger'

-- Jonathan Tompson
-- NYU, MRL
-- This script turns the serialized neural network file into a file that is
-- readable by my c++ code.

model_filename = "results/handmodel_fullcoeffs_tanh_abs_mid_L4Pooling.net"

-- Load in the settings file
dofile("load_settings.lua")  -- We don't need to do this

-- Load in the serialized convent
model = torch.load(model_filename)

-- Open an output file
convnet = torch.DiskFile(model_filename .. ".convnet", 'w')
convnet:binary()

-- ***************************************************
-- Save top level meta data
-- 1. Number of convolution stages
convnet:writeInt(2)
-- 2. Number of Neural Network stages
convnet:writeInt(2)

-- ***************************************************
-- Save the first conv stage.
stg1 = model:get(1)
-- image.display{image=stg1.weight, padding=2, zoom=4}
saveConvStage(stg1, stg1_norm, stg1_poolsizeu, stg1_pooling, stg1_nonlinear, 
              convnet)

-- ***************************************************
-- Save the second conv stage.
stg2 = model:get(5)
saveConvStage(stg2, stg2_norm, stg2_poolsizeu, stg2_pooling, stg2_nonlinear, 
              convnet)

-- ***************************************************
-- Save the first neural net stage.
stg1 = model:get(10)
stg1_nonlinear = "Tanh"
saveNNStage(stg1, stg1_nonlinear, convnet)

-- ***************************************************
-- Save the second neural net stage.
stg2 = model:get(12)
stg2_nonlinear = "None"
saveNNStage(stg2, stg2_nonlinear, convnet)

print("All done saving convnet")
