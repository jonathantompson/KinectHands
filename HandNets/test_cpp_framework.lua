require 'nn'
require 'image'
require 'torch'
require 'optim'
require 'sys'
torch.setdefaulttensortype('torch.FloatTensor')

num_feats_in = 4
num_feats_out = 6
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

-- Test Linear
-- TO DO:

-- Test SpatialLPPooling
pool_order = 2
nstates = num_feats_in
poolsize_u = 2
poolsize_v = 2
model:add(nn.SpatialLPPooling(nstates, pooling, poolsize_u, poolsize_v, poolsize_u, poolsize_v))




