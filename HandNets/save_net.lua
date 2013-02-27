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

model_filename = "results/handmodel.net"

-- Load in the settings file
print("--> Loading settings from file")
dofile("load_settings.lua")

-- Load in the serialized convent
print("--> Loading model from file")
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
-- 3. Number of banks
convnet:writeInt(num_hpf_banks)
-- 4. Input data type
convnet:writeInt(use_hpf_depth)

for j=1,num_hpf_banks do
  print(string.format("--> Saving bank %d convnet stage 1 and 2", j))
  -- ***************************************************
  -- Save the first conv stage.
  stg1 = model:get(1):get(j):get(1)
  -- image.display{image=stg1.weight, padding=2, zoom=4}
  saveConvStage(stg1, norm, poolsize[j][1], pooling, nonlinear, convnet)

  -- ***************************************************
  -- Save the second conv stage.
  if (poolsize[j][1] == 1) then  -- no pooling in the first stage
    stg2 = model:get(1):get(j):get(4)
  else
    stg2 = model:get(1):get(j):get(5)
  end
  saveConvStage(stg2, norm, poolsize[j][2], pooling, nonlinear, convnet)
end

-- ***************************************************
-- Save the first neural net stage.
print("--> Saving neural net stage 1")
stg1 = model:get(3)
saveNNStage(stg1, nonlinear, convnet)

-- ***************************************************
-- Save the second neural net stage.
print("--> Saving neural net stage 2")
stg2 = model:get(5)
stg2_nn_nonlinear = "None"  -- none
saveNNStage(stg2, stg2_nn_nonlinear, convnet)

print("All done saving convnet")
