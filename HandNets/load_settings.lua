settings_filename = model_filename .. '.settings.csv'

set_file = torch.DiskFile(settings_filename, 'r')
set_file:ascii()

headerLine = set_file:readString("*l")
dataLine = set_file:readString("*l")

-- Load in required functions
dofile("ParseCSVLine.lua")
dofile("TrimTrailLeadWhitespace.lua")

dataLineParsed = ParseCSVLine(dataLine)

-- Now get out the data
ind = 1
width = tonumber(dataLineParsed[ind])
ind = ind + 1
height = tonumber(dataLineParsed[ind])
ind = ind + 1
trsize = tonumber(dataLineParsed[ind])
ind = ind + 1
tesize = tonumber(dataLineParsed[ind])
ind = ind + 1
num_coeff = tonumber(dataLineParsed[ind])
ind = ind + 1
num_hpf_banks = tonumber(dataLineParsed[ind])
ind = ind + 1
use_hpf_depth = tonumber(dataLineParsed[ind])
ind = ind + 1

nonlinear = -1
nstates = {}
filtsize = {}
poolsize = {}
for j=1,num_hpf_banks do
  -- Stage 1
  stg1_filtsize = tonumber(dataLineParsed[ind])
  ind = ind + 1
  stg1_connect = trim(dataLineParsed[ind])
  ind = ind + 1
  stg1_nstates = tonumber(dataLineParsed[ind])
  ind = ind + 1
  stg1_nonlinear = trim(dataLineParsed[ind])
  ind = ind + 1
  if (nonlinear == -1) then
    nonlinear = stg1_nonlinear
  else
    if (nonlinear ~= stg1_nonlinear) then
      print('ERROR: stg1_nonlinear ~= nonlinear!')
    end
  end
  stg1_poolsettings = trim(dataLineParsed[ind])
  ind = ind + 1
  stg1_poolsizeu, stg1_poolsizev = 
    stg1_poolsettings:match("(%d+)x(%d+)") 
  stg1_poolsizeu = tonumber(stg1_poolsizeu)
  stg1_poolsizev = tonumber(stg1_poolsizev)
  
  -- Stage 2
  stg2_filtsize = tonumber(dataLineParsed[ind])
  ind = ind + 1
  stg2_connect = trim(dataLineParsed[ind])
  ind = ind + 1
  stg2_nstates = tonumber(dataLineParsed[ind])
  ind = ind + 1
  stg2_nonlinear = trim(dataLineParsed[ind])
  ind = ind + 1
  if (nonlinear ~= stg2_nonlinear) then
    print('ERROR: stg2_nonlinear ~= nonlinear!')
  end
  stg2_poolsettings = trim(dataLineParsed[ind])
  ind = ind + 1
  stg2_poolsizeu, stg2_poolsizev, stg2_pooling = 
    stg2_poolsettings:match("(%d+)x(%d+)") 
  stg2_poolsizeu = tonumber(stg2_poolsizeu)
  stg2_poolsizev = tonumber(stg2_poolsizev)
  
  table.insert(nstates, {stg1_nstates, stg2_nstates})
  table.insert(filtsize, {stg1_filtsize, stg2_filtsize})
  -- assume poolsizeu = poolsizev
  table.insert(poolsize, {stg1_poolsizeu, stg2_poolsizeu})
end

-- Neural net
nn_connect1 = trim(dataLineParsed[ind])
ind = ind + 1
nn_states1 = tonumber(dataLineParsed[ind])
ind = ind + 1
nn_connect2 = trim(dataLineParsed[ind])
ind = ind + 1
nn_states2 = num_coeff

loss_func = trim(dataLineParsed[ind])
ind = ind + 1
learning_rate = tonumber(dataLineParsed[ind])
ind = ind + 1
l2_reg_param = tonumber(dataLineParsed[ind])
ind = ind + 1
learning_momentum = tonumber(dataLineParsed[ind])
ind = ind + 1
learning_rate_decay = tonumber(dataLineParsed[ind])
ind = ind + 1
max_num_epochs = tonumber(dataLineParsed[ind])
ind = ind + 1
batch_size = tonumber(dataLineParsed[ind])
ind = ind + 1
