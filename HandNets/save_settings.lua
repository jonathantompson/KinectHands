settings_filename = model_filename .. '.settings.csv'

set_file = torch.DiskFile(settings_filename, 'w')
set_file:ascii()

-- Write the headers

-- Input settings header
set_file:writeString('W,H,#tr,#te,#coeff,num_banks,hpf_depth')
for j=1,num_hpf_banks do
  -- Stage 1 settings header
  set_file:writeString('filt_rad,fan-in,#feat,non-lin,norm.,pooling,')
  -- Stage 2 settings header
  set_file:writeString('filt_rad,fan-in,#feat,non-lin,norm.,pooling,')
end
-- Neural Net header
set_file:writeString('STG1conn.,STG2size,STG2conn,')
-- Training settings header
set_file:writeString('Loss,learningRate,weightDecay,momentum,')
set_file:writeString('learningRateDecay,')
set_file:writeString('\n')

-- Write the data

-- Input settings
set_file:writeString(string.format('%d,%d,', width, height))
set_file:writeString(string.format('%d,%d,', trainData.size(),
  testData.size()))
set_file:writeString(string.format('%d,', num_coeff))
set_file:writeString(string.format('%d,', num_hpf_banks))
set_file:writeString(string.format('%d,', use_hpf_depth))

-- Stage 1 settings
for j=1,num_hpf_banks do
  set_file:writeString(string.format('%d,', filtsize[j][1]))
  set_file:writeString('full,')
  set_file:writeString(string.format('%d,', nstates[j][1]))
  if (nonlinear == 1) then 
    set_file:writeString('SoftShrink,')
  elseif (nonlinear == 0) then
    set_file:writeString('Tanh,')
  end
  set_file:writeString('spac_sub.,')
  if (pooling ~= math.huge) then
    set_file:writeString(string.format('%dx%d L%d,', poolsize[j][1], 
      poolsize[j][1], pooling))
  else
    set_file:writeString(string.format('%dx%d Linf,', poolsize[j][1], 
      poolsize[j][1]))
  end

  -- Stage 2 settings
  set_file:writeString(string.format('%d,', filtsize[j][2]))
  set_file:writeString(string.format('%d rand,', fanin[j][1]))
  set_file:writeString(string.format('%d,', nstates[j][2]))
  if (nonlinear == 1) then 
    set_file:writeString('SoftShrink,')
  elseif (nonlinear == 0) then
    set_file:writeString('Tanh,')
  end
  set_file:writeString('spac_sub.,')
  if (pooling ~= math.huge) then
    set_file:writeString(string.format('%dx%d L%d,', poolsize[j][2], 
      poolsize[j][2], pooling))
  else
    set_file:writeString(string.format('%dx%d Linf,', poolsize[j][2], 
      poolsize[j][2]))
  end
end

-- Neural net settings
set_file:writeString('full,')
set_file:writeString(string.format('%d,', nstates_nn))
set_file:writeString('full,')

-- Training settings
if (loss == 0) then
  set_file:writeString('abs,')
elseif (loss == 1) then
  set_file:writeString('mse,')
else
  print("loss should be 0 or 1")
  return
end
set_file:writeString(string.format('%e,%e,%e,%e', optimState.learningRate, 
  optimState.weightDecay, optimState.momentum, optimState.learningRateDecay))

set_file:close()