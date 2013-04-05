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
  banks[j]:add(nn.SpatialConvolution(nfeats, nstates[j][1], filtsize[j][1], 
     filtsize[j][1]))
  -- SpatialConvolutionCUDA expects: height, width, batch, input_planes
  banks[j]:get(1).bias:add(-banks[j]:get(1).bias:min()) -- Set up the initial condition
  banks[j]:add(nn.Threshold())
  banks[j]:add(nn.SpatialMaxPooling(poolsize[j][1], poolsize[j][1], 
    poolsize[j][1], poolsize[j][1]))

  tensor_dim = {nstates[j][1], (tensor_dim[2] - filtsize[j][1] + 1) / 
    poolsize[j][1], (tensor_dim[3] - filtsize[j][1] + 1) / poolsize[j][1]}
  print(string.format("    Tensor Dimensions after stage 1 bank %d:", j))
  print(tensor_dim)

  -- Stage 2
  banks[j]:add(nn.SpatialConvolution(nstates[j][1], nstates[j][2], 
    filtsize[j][2], filtsize[j][2]))
  banks[j]:get(4).bias:add(-banks[j]:get(4).bias:min()) -- Set up the initial condition
  banks[j]:add(nn.Threshold())
  banks[j]:add(nn.SpatialMaxPooling(poolsize[j][2], poolsize[j][2], 
    poolsize[j][2], poolsize[j][2]))

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

model:add(nn.Linear(nstates_nn, num_coeff))

print("    Final output size")
print(num_coeff)

print '==> Converting model to cuda'
model:cuda()


if (false) then
  -- Test the SpatialConvolution Vs. SpatialConvolutionMap:
  data = torch.rand(128, 4, 96, 96):cuda()  -- batch, n_in, height, width
  n_in = 4
  n_out = 6
  filt_sizev = 5
  filt_sizeu = 5
  conv = nn.SpatialConvolution(n_in, n_out, filt_sizev, filt_sizeu):cuda()
  out = conv:forward(data)

  convCUDA = nn.Sequential()
  convCUDA:add(nn.Transpose({1,4}, {2,3}, {1,3}))  -- I'm sure this is wrong
  convCUDA:add(nn.SpatialConvolutionCUDA(n_in, n_out, filt_sizev, filt_sizeu))
  convCUDA:cuda()
  outCUDA = convCUDA:forward(data)
  print(outCUDA:size())

  input = data:clone()
  for _,perm in ipairs(convCUDA:get(1).permutations) do
    input = input:transpose(perm[1],perm[2])
  end

  convCUDA:add(nn.SpatialConvolutionCUDA(n_in, n_out, filt_sizev, filt_sizeu))
  convCUDA:add(nn.Transpose({1,3}, {2,4}, {1,2}, {1,4}))
  convCUDA:cuda()
  
end
