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
dofile("load_settings.lua")  -- We don't need to do this

-- Load in the serialized convent
model = torch.load(model_filename)

im_dir = "./hand_depth_data_processed/"

width = 96
height = 96
dim = width * height
num_coeff = 42
use_hpf_depth = 1
num_hpf_banks = 3

-- ******* Some preliminary calculations *********
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
    elseif string.find(files[i], "hpf_hands_") ~= nil then
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
depth_data = depth_file:readFloat(data_file_size)
depth_file:close()

depth = {}
ind = 1
for j=1,num_hpf_banks do
  table.insert(depth, torch.FloatTensor(1, bank_dim[j][1], bank_dim[j][2]))
  depth[j][{1, {}, {}}] = torch.FloatTensor(depth_data, ind, 
    torch.LongStorage{bank_dim[j][1], bank_dim[j][2]})
  ind = ind + (bank_dim[j][1]*bank_dim[j][2]) -- Move pointer forward
  depth[j] = depth[j]:double()
end

coeff = torch.FloatTensor(1, num_coeff)
coeff[{1,{}}] = torch.FloatTensor(coeff_data, 1,
  torch.LongStorage{num_coeff}):double()
coeff = coeff:double();

for j=1,num_hpf_banks do
  image.display{image=depth[j], padding=2, zoom=(3*math.pow(2,j)), legend='sample data point'}
end

output = model:forward(depth)

stg1_conv = model:get(1):get(1):get(1)

print("input data (40,40) --> (43,43)   (c++: 39 to 42)")
print(depth[1][{1,{40,43},{40,43}}])

stg1_conv_output = stg1_conv.output
print("stg1 conv out (40,40) --> (43,43) of feature 1 (c++: 39 to 42 of feat 0)")
print(stg1_conv_output[{1,{40,43},{40,43}}])
print("stg1 conv out (40,40) --> (43,43) of feature 2 (c++: 39 to 42 of feat 13)")
print(stg1_conv_output[{2,{40,43},{40,43}}])

--[[
-- Try manually convolving here
weights = stg1_conv.weight
bias = stg1_conv.bias
conn_table = stg1_conv.connTableRev
n_conv_output_features = 8
interm_w = 92
interm_h = 92
interm_dim = interm_w * interm_h
inw = 96
inh = 96
filt_fan_in = 1
in_dim = inw * inh
filt_height = 5
filt_width = 5
input = depth[1]

-- Initialize conv_output to the convolution bias
stg1_conv_output_man = torch.DoubleTensor(n_conv_output_features, interm_h, interm_w)
for i=1,n_conv_output_features do
  for v=1,interm_h do
    for u=1,interm_w do
      stg1_conv_output_man[{i,v,u}] = bias[{i}]
    end
  end
end

-- Now accumulate the connections by the correct weights into the conv_output
for outf=1,n_conv_output_features do
  xlua.progress(outf, n_conv_output_features)
  for inf=1,filt_fan_in do
    inf_index = conn_table[{outf,inf,1}]
    weight_index = conn_table[{outf,inf,2}]
    cur_filt = weights[{weight_index,{},{}}]

    for outv = 1,interm_h do
      for outu = 1,interm_w do
        -- Now perform the convlution of the inputs
        for filtv= 1,filt_height do
          for filtu = 1,filt_width do
            inu = outu + filtu - 1
            inv = outv + filtv - 1
            stg1_conv_output_man[{outf, outv, outu}] = stg1_conv_output_man[{outf, outv, outu}] + (cur_filt[{filtv, filtu}] * input[{inf_index, inv, inu}])
          end
        end
      end
    end
  end
end

print("stg1 conv out manual (40,40) --> (43,43) of feature 1")
print(stg1_conv_output_man[{1,{40,43},{40,43}}])
print("stg1 conv out manual (40,40) --> (43,43) of feature 2")
print(stg1_conv_output_man[{2,{40,43},{40,43}}])
--]]

stg1_non_linear = model:get(1):get(1):get(2)

stg1_nl_output = stg1_non_linear.output
print("stg1 nl out (40,40) --> (43,43) of feature 1 (c++: 39 to 42 of feat 0)")
print(stg1_nl_output[{1,{40,43},{40,43}}])
print("stg1 nl out (40,40) --> (43,43) of feature 2 (c++: 39 to 42 of feat 13)")
print(stg1_nl_output[{2,{40,43},{40,43}}])

stg1_pool = model:get(1):get(1):get(3)

stg1_pool_output = stg1_pool:get(3).output

print("stg1 pool out (20,20) --> (23,23) of feature 1 (c++: 19 to 22 of feat 0)")
print(stg1_pool_output[{1,{20,23},{20,23}}])
print("stg1 pool out (20,20) --> (23,23) of feature 2 (c++: 19 to 22 of feat 13)")
print(stg1_pool_output[{2,{20,23},{20,23}}])

stg1_norm = model:get(1):get(1):get(4)

stg1_norm_out = stg1_norm.output

print("stg1 norm out (20,20) --> (23,23) of feature 1 (c++: 19 to 22 of feat 0)")
print(stg1_norm_out[{1,{20,23},{20,23}}])
print("stg1 norm out (20,20) --> (23,23) of feature 2 (c++: 19 to 22 of feat 13)")
print(stg1_norm_out[{2,{20,23},{20,23}}])





stg2_conv = model:get(1):get(1):get(5)

stg2_conv_output = stg2_conv.output
print("stg2 conv out (18,18) --> (21,21) of feature 1 (c++: 17 to 20 of feat 0)")
print(stg2_conv_output[{1,{18,21},{18,21}}])
print("stg2 conv out (18,18) --> (21,21) of feature 2 (c++: 17 to 20 of feat 13)")
print(stg2_conv_output[{2,{18,21},{18,21}}])

--[[
-- Try manually convolving here
weights = stg2_conv.weight
bias = stg2_conv.bias
conn_table = stg2_conv.connTableRev
n_conv_output_features = 64
interm_w = 40
interm_h = 40
interm_dim = interm_w * interm_h
inw = 46
inh = 46
filt_fan_in = 4
in_dim = inw * inh
filt_height = 7
filt_width = 7
input = stg1_norm_out

-- Initialize conv_output to the convolution bias
stg2_conv_output_man = torch.DoubleTensor(n_conv_output_features, interm_h, interm_w)
for i=1,n_conv_output_features do
  for v=1,interm_h do
    for u=1,interm_w do
      stg2_conv_output_man[{i,v,u}] = bias[{i}]
    end
  end
end

-- Now accumulate the connections by the correct weights into the conv_output
for outf=1,n_conv_output_features do
  xlua.progress(outf, n_conv_output_features)
  for inf=1,filt_fan_in do
    inf_index = conn_table[{outf,inf,1}]
    weight_index = conn_table[{outf,inf,2}]
    cur_filt = weights[{weight_index,{},{}}]

    for outv = 1,interm_h do
      for outu = 1,interm_w do
        -- Now perform the convlution of the inputs
        for filtv= 1,filt_height do
          for filtu = 1,filt_width do
            inu = outu + filtu - 1
            inv = outv + filtv - 1
            stg2_conv_output_man[{outf, outv, outu}] = stg2_conv_output_man[{outf, outv, outu}] + (cur_filt[{filtv, filtu}] * input[{inf_index, inv, inu}])
          end
        end
      end
    end
  end
end

print("stg2 conv out man (18,18) --> (21,21) of feature 1 (c++: 17 to 20 of feat 0)")
print(stg2_conv_output_man[{1,{18,21},{18,21}}])
print("stg2 conv out man (18,18) --> (21,21) of feature 14 (c++: 17 to 20 of feat 13)")
print(stg2_conv_output_man[{14,{18,21},{18,21}}])
--]]

bank1_out_feats = model:get(1):get(1):get(8)
print("bank1_out_feats (3,3) --> (8,8) of feature 1 (c++: 2 to 7 of feat 0)")
print(bank1_out_feats.output[{1,{3,8},{3,8}}])

bank2_out_feats = model:get(1):get(2):get(8)
print("bank2_out_feats (3,3) --> (8,8) of feature 1 (c++: 2 to 7 of feat 0)")
print(bank2_out_feats.output[{1,{3,8},{3,8}}])

bank3_out_feats = model:get(1):get(3):get(7)
print("bank3_out_feats (3,3) --> (8,8) of feature 1 (c++: 2 to 7 of feat 0)")
print(bank3_out_feats.output[{1,{3,8},{3,8}}])

jointable = model:get(2)

first_stage_nnet = model:get(10)
