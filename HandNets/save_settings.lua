settings_filename = model_filename .. '.settings.csv'

set_file = torch.DiskFile(settings_filename, 'w')
set_file:ascii()

-- Write the headers

-- Input settings header
set_file:writeString('W, H, #tr, #te, #coeff,')
-- Stage 1 settings header
set_file:writeString('filt rad, fan-in, #feat, non-lin, norm., pooling,')
-- Stage 2 settings header
set_file:writeString('filt rad, fan-in, #feat, non-lin, norm., pooling,')
-- Neural Net header
set_file:writeString('STG1 conn., STG2 size, STG2 conn, ')
-- Training settings header
set_file:writeString('Loss, learningRate, weightDecay, momentum, learningRateDecay, ')
set_file:writeString('\n')

-- Write the data

-- Input settings
set_file:writeString(string.format('%d, %d, ', width, height))
-- set_file:writeString(string.format('%f, %f, ', mean_pix, std_pix))
set_file:writeString(string.format('%d, %d, ', trainData.size(), testData.size()))
set_file:writeString(string.format('%d, ', num_learned_coeff))

-- Stage 1 settings
set_file:writeString(string.format('%d, ', filtsize[1]))
set_file:writeString('full, ')
set_file:writeString(string.format('%d, ', nstates[1]))
if (nonlinear == 1) then 
  set_file:writeString('SoftShrink, ')
elseif (nonlinear == 0) then
  set_file:writeString('Tanh, ')
end
set_file:writeString('spac sub., ')
set_file:writeString(string.format('%dx%d L%d, ', poolsize[1], poolsize[1], pooling))

-- Stage 2 settings
set_file:writeString(string.format('%d, ', filtsize[2]))
set_file:writeString(string.format('%d rand, ', fanin[1]))
set_file:writeString(string.format('%d, ', nstates[2]))
if (nonlinear == 1) then 
  set_file:writeString('SoftShrink, ')
elseif (nonlinear == 0) then
  set_file:writeString('Tanh, ')
end
set_file:writeString('spac sub., ')
set_file:writeString(string.format('%dx%d L%d, ', poolsize[2], poolsize[2], pooling))

-- Neural net settings
set_file:writeString('full, ')
set_file:writeString(string.format('%d, ', nstates[3]))
set_file:writeString('full, ')

-- Training settings
if (loss == 0) then
  set_file:writeString('abs, ')
elseif (loss == 1) then
  set_file:writeString('mse, ')
else
  print("loss should be 0 or 1")
  return
end
set_file:writeString(string.format('%e, %e, %e, %e', optimState.learningRate, optimState.weightDecay, optimState.momentum, optimState.learningRateDecay))

set_file:close()