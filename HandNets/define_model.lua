-- ********************** Construct model **********************
print '==> Constructing model'

model = nn.Sequential()  -- top level model

-- define the seperate convnet banks (that will execute in parallel)
banks = {}
banks_total_output_size = 0
for j=1,num_hpf_banks do
  table.insert(banks, nn.Sequential())
  tensor_dim = {1, bank_dim[j][1], bank_dim[j][2]}

  -- TO DO: Switch to this:
  -- nn.SpatialConvolutionCUDA
  -- nn.Threshold
  -- nn.SpatialMaxPoolingCUDA
  
  -- Stage 1
  -- banks[j]:add(nn.SpatialConvolution(nfeats, nstates[j][1], 
  --   filtsize[j][1], filtsize[j][1]))
  banks[j]:add(nn.Transpose({1,4}, {1,3}, {1,2}))
  banks[j]:add(nn.SpatialConvolutionCUDA(nfeats, nstates[j][1],
    filtsize[j][1], filtsize[j][1]))
  conv = banks[j]:get(banks[j]:size())
  conv.bias:add(-conv.bias:min())  -- Add threshold initial condition
  banks[j]:add(nn.Transpose({1,2}, {1,3}, {1,4}))
  banks[j]:add(nn.Threshold())
  -- banks[j]:add(nn.SpatialMaxPooling(poolsize[j][1], poolsize[j][1]))
  banks[j]:add(nn.Transpose({1,4}, {1,3}, {1,2}))
  banks[j]:add(nn.SpatialMaxPoolingCUDA(poolsize[j][1], poolsize[j][1]))
  banks[j]:add(nn.Transpose({1,2}, {1,3}, {1,4}))

  tensor_dim = {nstates[j][1], (tensor_dim[2] - filtsize[j][1] + 1) / 
    poolsize[j][1], (tensor_dim[3] - filtsize[j][1] + 1) / poolsize[j][1]}
  print(string.format("    Tensor Dimensions after stage 1 bank %d:", j))
  print(tensor_dim)

  -- Stage 2
  -- banks[j]:add(nn.SpatialConvolution(nstates[j][1], nstates[j][2], 
  --   filtsize[j][2], filtsize[j][2]))
  banks[j]:add(nn.Transpose({1,4}, {1,3}, {1,2}))
  banks[j]:add(nn.SpatialConvolutionCUDA(nstates[j][1], nstates[j][2],
    filtsize[j][2], filtsize[j][2]))
  conv = banks[j]:get(banks[j]:size())
  conv.bias:add(-conv.bias:min())  -- Add threshold initial condition
  banks[j]:add(nn.Transpose({1,2}, {1,3}, {1,4}))
  banks[j]:add(nn.Threshold())
  -- banks[j]:add(nn.SpatialMaxPooling(poolsize[j][2], poolsize[j][2]))
  banks[j]:add(nn.Transpose({1,4}, {1,3}, {1,2}))
  banks[j]:add(nn.SpatialMaxPoolingCUDA(poolsize[j][2], poolsize[j][2]))
  banks[j]:add(nn.Transpose({1,2}, {1,3}, {1,4}))

  tensor_dim = {nstates[j][2], (tensor_dim[2] - filtsize[j][2] + 1) / 
    poolsize[j][2], (tensor_dim[3] - filtsize[j][2] + 1) / poolsize[j][2]}
  print(string.format("    Tensor Dimensions after stage 2 bank %d:", j))
  print(tensor_dim)

  vec_length = tensor_dim[1] * tensor_dim[2] * tensor_dim[3]
  banks[j]:add(nn.Reshape(vec_length))
  banks_total_output_size = banks_total_output_size + vec_length

  print(string.format("    Bank %d output length:", j))
  print(vec_length)
end

-- Now join the banks together!
-- Parallel applies ith member module to the ith input, and outpus a table
parallel = nn.ParallelTable()
for j=1,num_hpf_banks do
  parallel:add(banks[j])
end
model:add(parallel)
-- If we're doing mini-batches then we need to join along dim=2
-- otherwise we need to join along dim=1
model:add(nn.JoinTable(2))  -- Take the table of tensors and concat them

-- stage 3 : standard 2-layer neural network

print("    Neural net first stage input size")
print(banks_total_output_size)

model:add(nn.Linear(banks_total_output_size, nstates_nn))
model:get(3).bias:add(-model:get(3).bias:min()) -- Set up the initial condition
model:add(nn.Threshold())

print("    Neural net first stage output size")
print(nstates_nn)

-- EXTRA LINEAR LAYER
-- model:add(nn.Linear(nstates_nn, nstates_nn))
-- model:get(5).bias:add(-model:get(5).bias:min()) -- Set up the initial condition
-- model:add(nn.Threshold())
-- print("    Neural net second stage output size")
-- print(nstates_nn)
-- END EXTRA LINEAR LAYER

model:add(nn.Linear(nstates_nn, num_coeff))

print("    Final output size")
print(num_coeff)

print '==> Converting model to cuda'
model:cuda()

-- Temp code to test data
if (false) then
  -- Test the SpatialConvolution Vs. SpatialConvolutionCUDA:
  data = torch.rand(128, 4, 96, 96):cuda()  -- batch, n_in, height, width
  n_in = 4
  n_out = 16
  filt_sizev = 5
  filt_sizeu = 5
  conv = nn.SpatialConvolution(n_in, n_out, filt_sizev, filt_sizeu):cuda()
  out = conv:forward(data)

  convCUDA = nn.Sequential()
  convCUDA:add(nn.Transpose({1,4}, {1,3}, {1,2}))
  tmp = nn.SpatialConvolutionCUDA(n_in, n_out, filt_sizev, filt_sizeu)
  tmp:copy(conv)
  convCUDA:add(tmp)
  convCUDA:add(nn.Transpose({1,2}, {1,3}, {1,4}))
  convCUDA:cuda()
  outCUDA = convCUDA:forward(data)

  outCUDA = outCUDA:float()
  out = out:float()
  
  err = out - outCUDA
  torch.cdiv(err, out)
  err = err:abs()
  print "max CUDA Vs. Non-CUDA error/out is:"
  print(err:max())

  poolv = 2
  poolu = 2

  pool = nn.SpatialMaxPooling(poolv, poolu):cuda()
  out = pool:forward(data)

  poolCUDA = nn.Sequential()
  poolCUDA:add(nn.Transpose({1,4}, {1,3}, {1,2}))
  tmp = nn.SpatialMaxPoolingCUDA(poolv, poolu)
  poolCUDA:add(tmp)
  poolCUDA:add(nn.Transpose({1,2}, {1,3}, {1,4}))
  poolCUDA:cuda()
  outCUDA = poolCUDA:forward(data)

  outCUDA = outCUDA:float()
  out = out:float()

  err = out - outCUDA
  torch.cdiv(err, out)
  err = err:abs()
  print "max CUDA Vs. Non-CUDA error/out is:"
  print(err:max())

end
