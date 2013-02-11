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

model_filename = "results/handmodel_fullcoeffs_tanh_abs_mid_L2Pooling.net"

-- Load in the settings file
dofile("load_settings.lua")  -- We don't need to do this

-- Load in the serialized convent
model = torch.load(model_filename)

im_dir = "./hand_depth_data_processed/"

width = 96
height = 96
dim = width * height
num_coeff = 25  -- Keep this at 25!
num_learned_coeff = 25

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

cur_image = 1

-- ************ Get Data locations ***************
-- Read in the sample
print("loading in " .. depth_files[cur_image])

coeff_file = torch.DiskFile(im_dir .. coeffr_files[cur_image], 'r')
coeff_file:binary()
coeff_data = coeff_file:readFloat(num_coeff)
coeff_file:close()

depth_file = torch.DiskFile(im_dir .. depth_files[cur_image], 'r')
depth_file:binary()
depth_data = depth_file:readFloat(dim)
depth_file:close()

depth = torch.FloatTensor(1, height, width)
depth[{1, {}, {}}] = torch.FloatTensor(depth_data, 1, torch.LongStorage{height, width})
coeff = torch.FloatTensor(1, num_learned_coeff)
coeff[{1,{}}] = torch.FloatTensor(coeff_data, num_coeff - num_learned_coeff + 1,
  torch.LongStorage{num_learned_coeff}):double()

image.display{image=depth, padding=2, zoom=3, legend='sample data point'}

depth = depth:double();
coeff = coeff:double();
output = model:forward(depth)

stg1_conv = model:get(1)

print("input data (40,40) --> (43,43)   (c++: 39 to 42)")
print(depth[{1,{40,43},{40,43}}])