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
width = tonumber(dataLineParsed[1])
height = tonumber(dataLineParsed[2])
trsize = tonumber(dataLineParsed[3])
tesize = tonumber(dataLineParsed[4])
num_learned_coeff = tonumber(dataLineParsed[5])

-- Stage 1
stg1_filtsize = tonumber(dataLineParsed[6])
stg1_connect = trim(dataLineParsed[7])
stg1_nstates = tonumber(dataLineParsed[8])
stg1_nonlinear = trim(dataLineParsed[9])
stg1_norm = trim(dataLineParsed[10])
stg1_poolsettings = trim(dataLineParsed[11])
stg1_poolsizeu, stg1_poolsizev, stg1_pooling = stg1_poolsettings:match("(%d+)x(%d+) L(%d+)") 

-- Stage 2
stg2_filtsize = tonumber(dataLineParsed[12])
stg2_connect = trim(dataLineParsed[13])
stg2_nstates = tonumber(dataLineParsed[14])
stg2_nonlinear = trim(dataLineParsed[15])
stg2_norm = trim(dataLineParsed[16])
stg2_poolsettings = trim(dataLineParsed[17])
stg2_poolsizeu, stg2_poolsizev, stg2_pooling = stg2_poolsettings:match("(%d+)x(%d+) L(%d+)") 

-- Neural net
nn_connect1 = trim(dataLineParsed[18])
nn_states1 = tonumber(dataLineParsed[19])
nn_connect2 = trim(dataLineParsed[20])
nn_states2 = num_learned_coeff

loss_func = trim(dataLineParsed[21])
learning_rate = tonumber(dataLineParsed[22])
learning_weight_decay = tonumber(dataLineParsed[23])
learning_momentum = tonumber(dataLineParsed[24])
learning_rate_decay = tonumber(dataLineParsed[25])
