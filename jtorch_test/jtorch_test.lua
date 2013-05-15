require 'nn'
require 'image'
require 'torch'
require 'optim'
require 'sys'
torch.setdefaulttensortype('torch.FloatTensor')

num_feats_in = 5
num_feats_out = 10
width = 10
height = 10

data_in = torch.FloatTensor(num_feats_in, height, width)

for f=1,num_feats_in do
  val = f - (width * height) / 16
  for v=1,height do
    for u=1,width do
      data_in[{f,v,u}] = val
      val = val + 1/8
    end
  end
end

print('Data In:')
print(data_in)

model = nn.Sequential()

-- Test tanh
model:add(nn.Tanh())
res = model:forward(data_in)
print('Tanh result')
print(res)

-- Test Threshold
threshold = 0.5
val = 0.1
model:add(nn.Threshold(threshold, val))
res = model:forward(data_in)
print('Threshold result')
print(res)

-- Test SpatialConvolutionMap
n_states_in = num_feats_in
n_states_out = num_feats_out
fan_in = 3
filt_width = 5
filt_height = 5
conn_table = nn.tables.random(n_states_in, n_states_out, fan_in)
for f_out=1,num_feats_out do
  for f_in=1,fan_in do
    conn_table[{(f_out-1) * fan_in + f_in, 2}] = f_out
    cur_f_in = math.mod((f_out-1) + (f_in-1), n_states_in) + 1
    conn_table[{(f_out-1) * fan_in + f_in, 1}] = cur_f_in
  end
end
print('Spatial Convolution Map Connection Table')
print(conn_table)
spat_conv_map = nn.SpatialConvolutionMap(conn_table, filt_width, filt_height)
print('Spatial Convolution Map Connection Table Rev')
print(spat_conv_map.connTableRev)
for f_out=1,num_feats_out do
  spat_conv_map.bias[{f_out}] = f_out / num_feats_out - 0.5
end
print('Spatial Convolution Map Biases')
print(spat_conv_map.bias)
num_filt = fan_in * num_feats_out
sigma_x_sq = 1
sigma_y_sq = 1
for filt=1,num_filt do
  for v=1,filt_height do
    for u=1,filt_width do
      x = u - 1 - (filt_width-1) / 2
      y = v - 1 - (filt_height-1) / 2
      spat_conv_map.weight[{filt, v, u}] = (filt / num_filt) * math.exp(-((x*x) / (2*sigma_x_sq) + 
        (y*y) / (2*sigma_y_sq)))
    end
  end
end
print('Spatial Convolution Map Weights')
print(spat_conv_map.weight)
model:add(spat_conv_map)
res = model:forward(data_in)
print('Spatial Convolution Map result')
print(res)

-- Test SpatialConvolution
n_states_in = num_feats_in
n_states_out = num_feats_out
fan_in = n_states_in
filt_width = 5
filt_height = 5
spat_conv = nn.SpatialConvolution(n_states_in, n_states_out, filt_width, filt_height)
for f_out=1,num_feats_out do
  spat_conv.bias[{f_out}] = f_out / num_feats_out - 0.5
end
print('Spatial Convolution Biases')
print(spat_conv.bias)
num_filt = num_feats_out * num_feats_in
sigma_x_sq = 1
sigma_y_sq = 1
for fout=1,num_feats_out do
  for fin=1,num_feats_in do
    for v=1,filt_height do
      for u=1,filt_width do
        filt = (fout-1) * num_feats_out + (fin - 1) + 1
        x = u - 1 - (filt_width-1) / 2
        y = v - 1 - (filt_height-1) / 2
        spat_conv.weight[{fout, fin, v, u}] = (filt / num_filt) * math.exp(-((x*x) / (2*sigma_x_sq) + 
          (y*y) / (2*sigma_y_sq)))
      end
    end
  end
end
print('Spatial Convolution Weights')
print(spat_conv.weight)
res = spat_conv:forward(model:get(2).output)
print('Spatial Convolution result')
print(res)

-- Test SpatialLPPooling
pnorm = 2.0
nstates = num_feats_out
poolsize_u = 2
poolsize_v = 2
pool_stage = nn.SpatialLPPooling(nstates, poolsize_u, poolsize_v, poolsize_u, poolsize_v)
model:add(pool_stage)
res = model:forward(data_in)
print('SpatialLPPooling result')
print(res)

-- Test SpatialMaxPooling
model3 = nn.Sequential()
max_pool_stage = nn.SpatialMaxPooling(poolsize_u, poolsize_v, poolsize_u, poolsize_v)
model3:add(max_pool_stage)
res = model3:forward(data_in)
print('SpatialMaxPooling result')
print(res)

-- Test SpatialSubtractiveNormalization
model4 = nn.Sequential()
normkernel = image.gaussian1D(7)
print('Normalization Kernel1D')
print(normkernel)
spatial_sub_norm = nn.SpatialSubtractiveNormalization(num_feats_in, normkernel)
model4:add(spatial_sub_norm)
res = model4:forward(data_in)
print('SpatialSubtractiveNormalization result')
print(res)

-- Test SpatialDivisiveNormalization
model5 = nn.Sequential()
normkernel = image.gaussian1D(7)
print('Normalization Kernel1D')
print(normkernel)
spatial_div_norm = nn.SpatialDivisiveNormalization(num_feats_in, normkernel)
model5:add(spatial_div_norm)
res = model5:forward(data_in)
print('SpatialDivisiveNormalization result')
print(res)
-- return spatial_div_norm.localstds

-- Test SpatialContrastiveNormalization
model6 = nn.Sequential()
lena = image.rgb2y(image.lena()):float()
file = torch.DiskFile("lena_image.bin", 'w')
file:binary()
for i=1,lena:size()[2] do
  for v=1,lena:size()[3] do
    file:writeFloat(lena[{1, i, v}])
  end
end
file:close()
normkernel = torch.Tensor(7):fill(1)
spatial_contrast_norm = nn.SpatialContrastiveNormalization(1, normkernel)
model6:add(spatial_contrast_norm)
res = model6:forward(lena)
image.display(lena)
image.display(res)
file = torch.DiskFile("lena_image_processed.bin", 'r')
file:binary()
lena_processed = file:readFloat(lena:size()[2] * lena:size()[3])
lena_processed = torch.FloatTensor(lena_processed, 1, torch.LongStorage{1, 
lena:size()[2], lena:size()[3]}):float()
file:close()
image.display(lena_processed)
err = lena_processed - res
err_abs = torch.abs(err)
image.display(err_abs)

-- Test the local contrast normalization of the hand image generator
normkernel = torch.Tensor(11):fill(1)
spatial_contrast_norm = nn.SpatialContrastiveNormalization(1, normkernel)
file = torch.DiskFile("processed_hands_4618452720732.bin", 'r')
file:binary()
depth = file:readFloat(96*96)
depth = torch.FloatTensor(depth, 1, torch.LongStorage{1, 96, 96}):float()
file:close()
file = torch.DiskFile("hpf_processed_hands_4618452720732.bin", 'r')
file:binary()
hpf_depth = file:readFloat(96*96)
hpf_depth = torch.FloatTensor(hpf_depth, 1, torch.LongStorage{1, 96, 96}):float()
file:close()
image.display{image=hpf_depth, zoom=(6.0)}
image.display{image=depth, zoom=(6.0)}
hpf_depth_torch = spatial_contrast_norm:forward(depth)
image.display{image=hpf_depth_torch, zoom=(6.0)}

-- Test Linear
model2 = nn.Sequential()
lin_size = num_feats_in * width * height
lin_size_out = 20
model2:add(nn.Reshape(lin_size))
lin_stage = nn.Linear(lin_size, lin_size_out)
for i=1,lin_size_out do
  for j=1,lin_size do
    k = (i-1) * lin_size + j
    lin_stage.weight[{i,j}] = k / (lin_size * lin_size_out)
  end
end
for i=1,lin_size_out do
  lin_stage.bias[{i}] = i / lin_size_out
end
model2:add(lin_stage)
res = model2:forward(data_in)
print('Linear result')
print(res)

-- Test model
require 'nn'
require 'cunn'
require 'image'
require 'torch'
require 'cutorch'
require 'optim'
require 'sys'
require 'xlua'
torch.setdefaulttensortype('torch.FloatTensor')
model = torch.load('handmodel.net')
data_file_size = 0
bank_dim = {}
dim = 96
num_coeff = 40
num_hpf_banks = 3
for i = 1,num_hpf_banks do
  table.insert(bank_dim, {dim, dim})
  data_file_size = data_file_size + dim * dim
  dim = dim / 2
end

hpf_depth_file = torch.DiskFile('kinect_hpf_depth_image_uncompressed.bin','r')
hpf_depth_file:binary()
hpf_depth_data = hpf_depth_file:readFloat(data_file_size)
hpf_depth_file:close()

data = {}
input = {
  data = {},
  labels = torch.FloatTensor(1, num_coeff),
  size = function() return 1 end
}
ind = 1
for j=1,num_hpf_banks do
  table.insert(data, torch.FloatTensor(hpf_depth_data, ind, 
    torch.LongStorage{bank_dim[j][1], bank_dim[j][2]}):float())
  ind = ind + (bank_dim[j][1]*bank_dim[j][2]) -- Move pointer forward
  table.insert(input.data, torch.FloatTensor(1, 1, 
    bank_dim[j][1], bank_dim[j][2]))
  input.data[j][{1,1,{},{}}] = data[j]
  input.data[j] = input.data[j]:cuda()
end

input.labels = model:forward(input.data)

print(input.labels)

-- Some non-zero outputs
join_table = model:get(2)
return join_table.output[{1,{501,600}}]

dofile('visualize_data.lua')  -- Just define the function
VisualizeData(input, visualize_data_labels)



