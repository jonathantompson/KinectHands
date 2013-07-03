function sendDatabase(dst, database, num_hpf_banks)
  local packet
  packet = { size = database:size() }
  dst:send(packet)
  packet = { heat_maps = database.heat_maps }
  dst:send(packet)
  packet = { labels = database.labels }
  dst:send(packet) 
  packet = { num_hpf_banks = num_hpf_banks }
  dst:send(packet) 
  for i = 1,num_hpf_banks do
    packet = { data = database.data[i] }
    dst:send(packet) 
  end
  packet = nil
  collectgarbage()
end

function recieveDatabase( src )
  local packet
  packet = src:receive()
  local trsize = packet.size

  local packet_heat_maps
  packet_heat_maps = src:receive()

  local packet_labels
  packet_labels = src:receive()

  packet = src:receive()
  local num_hpf_banks = packet.num_hpf_banks
  
  local database = {
    data = { },
    labels = packet_labels.labels:clone(),
    size = function() return trsize end,
    heat_maps = packet_heat_maps.heat_maps:clone()
  }

  local packet_data
  for i = 1,num_hpf_banks do
    packet_data = src:receive()
    table.insert(database.data, packet_data.data:clone())
  end

  return database, num_hpf_banks, trsize
end

