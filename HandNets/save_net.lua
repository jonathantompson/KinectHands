require 'nn'
require 'cunn'
require 'image'
require 'torch'
require 'cutorch'
require 'optim'   -- an optimization package, for online and batch methods
torch. setnumthreads(8)

jtorch_root = "../jtorch/"
dofile("../jtorch/jtorch.lua")

-- Jonathan Tompson
-- NYU, MRL
-- This script turns the serialized neural network file into a file that is
-- readable by my c++ code.

model_filename = "handmodel.net"

-- Load in the serialized convent
print("--> Loading model from file")
model = torch.load(model_filename)
model = model:float()

out_model_filename = model_filename .. ".convnet"

saveModel(model, out_model_filename)

print("All done saving convnet")

