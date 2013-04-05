require 'nn'
require 'cunn'
require 'image'
require 'torch'
require 'cutorch'
require 'optim'   -- an optimization package, for online and batch methods
torch. setnumthreads(8)

dofile("save_nn_node.lua")  -- Load in helper function

-- Jonathan Tompson
-- NYU, MRL
-- This script turns the serialized neural network file into a file that is
-- readable by my c++ code.

model_filename = "handmodel.net"

-- Load in the settings file
print("--> Loading settings from file")
dofile("load_settings.lua")

-- Load in the serialized convent
print("--> Loading model from file")
model = torch.load(model_filename)
model = model:float()

-- Open an output file
ofile = torch.DiskFile(model_filename .. ".convnet", 'w')
ofile:binary()

-- Now recursively save the network
saveNNNode(model, ofile)

ofile:close()

print("All done saving convnet")

if false then
  input = {}
  table.insert(input, torch.FloatTensor(4, 1, 96, 96):zero())
  table.insert(input, torch.FloatTensor(4, 1, 96/2, 96/2):zero())
  table.insert(input, torch.FloatTensor(4, 1, 96/4, 96/4):zero())
end
