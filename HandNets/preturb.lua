

-- Top level function, preturbs the whole database
-- Sits in another worker thread
function preturbThread()
  -- a worker starts with a blank stack, we need to reload our libraries 
  print('preturbThread(): starting thread...')
  require('torch')
  require('image')
  require('parallel')

  torch.setnumthreads(12)
  torch.manualSeed(1)

  -- Redefine helper functions
  dofile("distort.lua")
  dofile("preturb_data_and_labels.lua")
  dofile('preturb_send_recieve.lua')
  dofile("pbar.lua")

  -- recieve the database from the parent
  print('preturbThread(): waiting for "data"...')
  database, num_hpf_banks, trsize = recieveDatabase( parallel.parent )
  print('preturbThread(): database recieved, num_hpf_banks = ' .. num_hpf_banks 
    .. ', trsize = ' .. trsize)

  local data_rotated = {
    data = { },
    labels = database.labels:clone():float(),
    size = function() return trsize end,
    heat_maps = database.heat_maps:clone():float(),
  }

  for i=1,num_hpf_banks do
    table.insert(data_rotated.data, database.data[i]:clone():float())
  end

  print('preturbThread(): starting loop...')

  while true do
    collectgarbage()

    m = parallel.yield()
    if m == 'break' then break end

    -- Preturb the database
    for j=1,data_rotated:size() do
      local deg_rot
      local scale
      local transv
      local transu

      -- rotate the first bank and get back the rotation matrix
      data_rotated.data[1][{j,{},{},{}}], data_rotated.labels[{j,{}}], 
        data_rotated.heat_maps[{j,{},{},{}}], deg_rot, scale, transv, transu = 
        preturbDataAndLabels(database.data[1][{j,{},{},{}}], database.labels[{j,{}}],
        database.heat_maps[{j,{},{},{}}])
      -- Now rotate the other banks by the same transformation
      for b=2,num_hpf_banks do
        data_rotated.data[b][{j,{},{},{}}] = 
          preturbDataAndLabels(database.data[b][{j,{},{},{}}], nil, nil, deg_rot, scale, 
            transv, transu)
      end
    end

    -- send some data back
    sendDatabase(parallel.parent, data_rotated, num_hpf_banks)
  end
  
  print('preturbThread(): quiting...')
end

function preturbManual(database, size)
  local t0 = sys.clock()

  dofile("distort.lua")
  dofile("preturb_data_and_labels.lua")
  dofile("pbar.lua")

  local data_rotated = {
    data = { },
    labels = database.labels:clone():float(),
    size = function() return size end,
    heat_maps = database.heat_maps:clone():float(),
  }
  num_hpf_banks = #database.data
  for i=1,num_hpf_banks do
    table.insert(data_rotated.data, database.data[i]:clone():float())
  end

  local next_progress = 100
  for j=1,data_rotated:size() do
    -- disp progress
    if (j >= next_progress or j == data_rotated:size()) then
      progress(j, data_rotated:size())
      next_progress = next_progress + 100
    end

    -- rotate the first bank and get back the rotation matrix
    local deg_rot
    local scale
    local transv
    local transu
    data_rotated.data[1][{j,{},{},{}}], data_rotated.labels[{j,{}}], data_rotated.heat_maps[{j,{},{},{}}], deg_rot, scale, transv, transu = preturbDataAndLabels(database.data[1][{j,{},{},{}}], database.labels[{j,{}}], database.heat_maps[{j,{},{},{}}])
    -- Now rotate the other banks by the same transformation
    for b=2,num_hpf_banks do
      data_rotated.data[b][{j,{},{},{}}] = 
        preturbDataAndLabels(database.data[b][{j,{},{},{}}], nil, nil, deg_rot, scale, 
          transv, transu)
    end
  end
  local t1 = sys.clock()
  print('Time taken to preturb database: ' .. (t1 - t0))

  return data_rotated
end
