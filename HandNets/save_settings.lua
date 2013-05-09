settings_filename = model_filename .. '.settings.csv'

set_file = torch.DiskFile(settings_filename, 'w')
set_file:ascii()

-- Write the headers

-- Input settings header
set_file:writeString('W,H,HMW,HMH,#tr,#te,#coeff,num_banks,hpf_depth')
for j=1,num_hpf_banks do
  -- Stage 1 settings header
  set_file:writeString('filt_rad,fan-in,#feat,non-lin,norm.,pooling,')
  -- Stage 2 settings header
  set_file:writeString('filt_rad,fan-in,#feat,non-lin,norm.,pooling,')
end
-- Neural Net header
set_file:writeString('STG1conn.,STG2size,STG2conn,')
-- Training settings header
set_file:writeString('Loss,learningRate,l2RegParam,momentum,')
set_file:writeString('learningRateDecay,maxEpochs,batchSize')
set_file:writeString('\n')

-- Write the data

-- Input settings
set_file:writeString(string.format('%d,%d,', width, height))
set_file:writeString(string.format('%d,%d,', heat_map_width, heat_map_height))
set_file:writeString(string.format('%d,%d,', trainData.size(), testData.size()))
set_file:writeString(string.format('%d,', num_coeff))
set_file:writeString(string.format('%d,', num_hpf_banks))
set_file:writeString(string.format('%d,', use_hpf_depth))

-- Stage 1 settings
for j=1,num_hpf_banks do
  set_file:writeString(string.format('%d,', filtsize[j][1]))
  set_file:writeString('full,')
  set_file:writeString(string.format('%d,', nstates[j][1]))
  set_file:writeString('Threshold,')
  set_file:writeString(string.format('%dx%d,', poolsize[j][1], 
    poolsize[j][1]))

  -- Stage 2 settings
  set_file:writeString(string.format('%d,', filtsize[j][2]))
  set_file:writeString('full,')
  set_file:writeString(string.format('%d,', nstates[j][2]))
  set_file:writeString('Threshold,')
  set_file:writeString(string.format('%dx%d,', poolsize[j][2], 
    poolsize[j][2]))
end

-- Neural net settings
set_file:writeString('full,')
nstates_nn = nstates_nn or -1
set_file:writeString(string.format('%d,', nstates_nn))
set_file:writeString('full,')

-- Training settings
set_file:writeString('mse,')

set_file:writeString(string.format('%e,%e,%e,%e', learning_rate, 
  l2_reg_param, learning_momentum, learning_rate_decay))
set_file:writeString(string.format('%d,%d,', max_num_epochs,
  batch_size))

set_file:close()

