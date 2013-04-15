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

torch.setnumthreads(8)
torch.manualSeed(1)

torch.setdefaulttensortype('torch.FloatTensor')
print("GPU That will be used:")
print(cutorch.getDeviceProperties(cutorch.getDevice()))

width = 96
height = 96
num_hpf_banks = 3
dim = width * height
num_coeff = 40
num_coeff_per_feature = 2  -- UV = 2, UVD = 3
frame_stride = 100  -- Only 1 works for now
perform_training = 1
model_filename = 'handmodel.net'
im_dir = "../data/hand_depth_data_processed_for_CN_synthetic/"
test_im_dir = "../data/hand_depth_data_processed_for_CN_test_synthetic/"
test_data_rate = 20  -- this means 1 / 20 FROM THE TRAINING SET will be test
use_hpf_depth = 1

-- ********************** Load data from Disk *************************
dofile('load_data.lua')

-- ************ Visualize one of the depth data samples ***************
visualize_data = 0
dofile('visualize_data.lua')  -- Just define the function:
-- VisualizeData(x, plot_labels, num_banks, n_tiles, zoom_factor)
if (visualize_data == 1) then
  VisualizeData(trainData)
  VisualizeData(testData)
end

dofile('preturb.lua')
trainDataRotated = preturb(trainData)

plot_labels = 1
num_banks = 3
n_tiles = 6
zoom_factor = 1
VisualizeData(trainData, plot_labels, num_banks, n_tiles, zoom_factor)
VisualizeData(trainDataRotated, plot_labels, num_banks, n_tiles, zoom_factor)



