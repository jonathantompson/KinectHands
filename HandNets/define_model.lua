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
model:add(nn.JoinTable(1))  -- Take the table of tensors and concat them

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

