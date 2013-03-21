require 'nn'
require 'image'
-- require 'torch'
require 'cutorch'
require 'optim'   -- an optimization package, for online and batch methods
require 'sys'
require 'xlua'  -- for profiler

dofile("pbar.lua")
dofile("shuffle_files.lua")
dofile("modules_cc.lua")

-- Initial setup
p = xlua.Profiler()
torch.setnumthreads(6)
torch.manualSeed(1)
torch.setdefaulttensortype('torch.FloatTensor')

num_banks = 1
tensor_dim = 1024
num_repeats = 10

-- ******************* FloatTensor ******************

data1 = torch.FloatTensor(num_banks, tensor_dim, tensor_dim)
for i=1,num_banks do
  data1[{i,{},{}}] = torch.rand(tensor_dim,tensor_dim):type('torch.FloatTensor')
end

model = nn.SpatialConvolution(1, 16, 10, 10)

-- p:start('spatialconv')
for j = 1,num_repeats do
  print(string.format("repeat %d of %d", j, num_repeats))
  time0 = sys.clock()
  output = model:forward(data1)
  time1 = sys.clock()
end
-- p:end('spatialconv')

print(string.format("Torch Tensor convoluation time: %.4f", (time1-time0)/num_repeats))

-- p:printAll{}

-- *********************** CUDA *********************

print("setting model and data to cuda")

require 'cunn'
cutorch.setDevice(1)
torch.setdefaulttensortype('torch.CudaTensor')

model = nn.SpatialConvolution(1, 16, 10, 10)
model = model:cuda()
data1_cuda = data1:type('torch.CudaTensor')

-- p:start('spatialconvCuda')
for j = 1,num_repeats do
  print(string.format("repeat %d of %d", j, num_repeats))
  time0 = sys.clock()
  output = model:forward(data1_cuda)
  cutorch.synchronize()
  time1 = sys.clock()
end
-- p:end('spatialconvCuda')

print(string.format("Torch Cuda Tensor convoluation time: %.4f", (time1-time0)/num_repeats))

-- p:printAll{}
