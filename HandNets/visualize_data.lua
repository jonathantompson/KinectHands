require 'image'

function round(num)
  local under = math.floor(num)
  local upper = math.floor(num) + 1
  local underV = -(under - num)
  local upperV = upper - num
  if (upperV > underV) then
    return under
  else
    return upper
  end
end

function VisualizeData(x, plot_labels, num_banks, n_tiles, zoom_factor)
  if (plot_labels == nil) then
    plot_labels = 1
  end
  n_tiles = n_tiles or 10
  num_banks = num_banks or num_hpf_banks
  local n_images = math.min(x.size(), n_tiles * n_tiles)
  zoom_factor = zoom_factor or 1
  for j=1,num_banks do
    local im = {
      data = x.data[j][{{1,n_images}, {}, {}, {}}],
    }
    im.data = im.data:double()
    if (j == 1 and plot_labels == 1) then
      im.labels = x.labels[{{1,n_images}, {}}]
      im.labels = im.labels:double()
      for i=1,n_images do
        local max_val = im.data[{i,{},{},{}}]:max()
        for k=1,num_coeff-1,num_coeff_per_feature do
          uv_pos = {
            round(bank_dim[j][1] * im.labels[{i, k}]), 
            round(bank_dim[j][2] * im.labels[{i, k+1}])
          }
          uv_pos[1] = math.min(uv_pos[1], bank_dim[j][1])
          uv_pos[2] = math.min(uv_pos[2], bank_dim[j][2])
          uv_pos[1] = math.max(uv_pos[1], 1)
          uv_pos[2] = math.max(uv_pos[2], 1)
          -- Add vert line at feature
          for v=(uv_pos[2]-1),(uv_pos[2]+1) do
            if (v >= 1 and v <= bank_dim[j][2]) then
              im.data[{i,1,v,uv_pos[1]}] = max_val
            end
          end
          -- Add horiz line at feature
          for u=(uv_pos[1]-1),(uv_pos[1]+1) do
            if (u >= 1 and u <= bank_dim[j][1]) then
              im.data[{i,1,uv_pos[2],u}] = max_val
            end
          end
        end
      end
    end
    image.display{image=im.data, padding=2, nrow=math.floor(math.sqrt(n_images)), 
      zoom=zoom_factor*(1*math.pow(2,j-1)), scaleeach=false}
  end
end

function VisualizeImage(x, index, plot_image, plot_labels, num_banks, n_tiles, zoom_factor)
  if (plot_labels == nil) then
    plot_labels = 1
  end
  if (plot_image == nil) then
    plot_image = 1
  end
  n_tiles = n_tiles or 10
  num_banks = num_banks or num_hpf_banks
  local n_images = 1
  zoom_factor = zoom_factor or 5
  local im = {
    data = torch.FloatTensor(num_banks, 1, height, width),
  }
  for j=1,num_banks do
    im.data[{j, {}, {}, {}}] = image.scale(x.data[j][{index, {}, {}, {}}]:float(), width, height,
      'simple')
  end

  if (plot_labels == 1) then
    im.labels = x.labels[{{index}, {}}]
    im.labels = im.labels:double()
    local max_val = im.data[{1,{},{},{}}]:max()
    for k=1,num_coeff-1,num_coeff_per_feature do
      uv_pos = {
        round(bank_dim[1][1] * im.labels[{1, k}]), 
        round(bank_dim[1][2] * im.labels[{1, k+1}])
      }
      uv_pos[1] = math.min(uv_pos[1], bank_dim[1][1])
      uv_pos[2] = math.min(uv_pos[2], bank_dim[1][2])
      uv_pos[1] = math.max(uv_pos[1], 1)
      uv_pos[2] = math.max(uv_pos[2], 1)
      -- Add vert line at feature
      for v=(uv_pos[2]-1),(uv_pos[2]+1) do
        if (v >= 1 and v <= bank_dim[1][2]) then
          im.data[{1,1,v,uv_pos[1]}] = max_val
        end
      end
      -- Add horiz line at feature
      for u=(uv_pos[1]-1),(uv_pos[1]+1) do
        if (u >= 1 and u <= bank_dim[1][1]) then
          im.data[{1,1,uv_pos[2],u}] = max_val
        end
      end
    end
  end
  if (plot_image == 1) then
    image.display{image=im.data, padding=2, nrow=3, zoom=zoom_factor, scaleeach=false}
  end
  
  -- Now display the heatmaps
  zoom_factor = 0.5 * zoom_factor * height / heat_map_height
  
  if (x.heat_maps:dim() == 4) then
    im.heat_maps = torch.Tensor(num_features, heat_map_height, heat_map_width):copy(
      x.heat_maps[{{index}, {}, {}, {}}])
  else
    im.heat_maps = torch.Tensor(num_features, heat_map_height, heat_map_width):copy(
      x.heat_maps[{{index}, {}}])
  end
  image.display{image=im.heat_maps, padding=2, nrow=4, zoom=zoom_factor,
    scaleeach=false}
end

function VisualizeHeatMap(x, index, hm_index)
  local im = {
    data = torch.FloatTensor(num_hpf_banks, 3, height, width),
  }
  for j=1,num_hpf_banks do
    im.data[{j, 1, {}, {}}] = image.scale(x.data[j][{index, {}, {}, {}}]:float(), width, height,
      'simple')
    local im_min = im.data[{j, 1, {}, {}}]:min()
    local im_max = im.data[{j, 1, {}, {}}]:max()
    im.data[{j, 1, {}, {}}]:add(-im_min)
    im.data[{j, 1, {}, {}}]:mul(1.0 / (im_max - im_min))
    im.data[{j, 2, {}, {}}] = im.data[{j, 1, {}, {}}]
    im.data[{j, 3, {}, {}}] = im.data[{j, 1, {}, {}}]
  end
  local hm_scaled = image.scale(x.heat_maps[{index, hm_index, {}, {}}]:float(), width, height,
      'simple')
  local hm_min = hm_scaled:min()
  local hm_max = hm_scaled:max()
  hm_scaled:add(-hm_min)
  hm_scaled:mul(1.0 / (hm_max - hm_min))  -- 0 to 1
  local val = hm_scaled
  val:mul(-1)
  val:add(1)  -- = (1 - hm_scaled)

  for j=1,num_hpf_banks do
    im.data[{j,2,{},{}}]:cmul(val)
    im.data[{j,3,{},{}}]:cmul(val)
  end

  image.display{image=im.data, padding=2, nrow=3, zoom=5, scaleeach=false}

end
