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
print('Spatial Convolution Connection Table')
print(conn_table)
spat_conv = nn.SpatialConvolutionMap(conn_table, filt_width, filt_height)
print('Spatial Convolution Connection Table Rev')
print(spat_conv.connTableRev)
for f_out=1,num_feats_out do
  spat_conv.bias[{f_out}] = f_out / num_feats_out - 0.5
end
print('Spatial Convolution Biases')
print(spat_conv.bias)
num_filt = fan_in * num_feats_out
sigma_x_sq = 1
sigma_y_sq = 1
for filt=1,num_filt do
  for v=1,filt_height do
    for u=1,filt_width do
      x = u - 1 - (filt_width-1) / 2
      y = v - 1 - (filt_height-1) / 2
      spat_conv.weight[{filt, v, u}] = (filt / num_filt) * math.exp(-((x*x) / (2*sigma_x_sq) + 
        (y*y) / (2*sigma_y_sq)))
    end
  end
end
print('Spatial Convolution Weights')
print(spat_conv.weight)
model:add(spat_conv)
res = model:forward(data_in)
print('Spatial Convolution Map result')
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
print('SpatialLPPooling result')
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
lena_processed = torch.FloatTensor(lena_processed, 1, torch.LongStorage{1, lena:size()[2], 
            lena:size()[3]}):float()
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
