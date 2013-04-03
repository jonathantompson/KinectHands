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


if (visualize_data == 1) then
  n_images = math.min(trainData.size(), 196)  -- 12 x 12
  for j=1,num_hpf_banks do
    im = {
      data = trainData.data[j][{{1,n_images}, {}, {}, {}}],
      labels = trainData.labels[{{1,n_images}, {}}]
    }
    im.data = im.data:double()
    im.labels = im.labels:double()
    if (j == 1 and visualize_data_labels == 1) then
      for i=1,n_images do
        max_val = im.data[{i,{},{},{}}]:max()
        for k=1,num_coeff-1,num_coeff_per_feature do
          uv_pos = {
            round(bank_dim[j][1] * im.labels[{i, k}]), 
            round(bank_dim[j][2] * im.labels[{i, k+1}])
          }
          -- Add vert line at feature
          for v=(uv_pos[2]-1),(uv_pos[2]+1) do
            im.data[{i,1,v,uv_pos[1]}] = max_val
          end
          -- Add horiz line at feature
          for u=(uv_pos[1]-1),(uv_pos[1]+1) do
            im.data[{i,1,uv_pos[2],u}] = max_val
          end
        end
      end
    end
    image.display{image=im.data, padding=2, nrow=math.floor(math.sqrt(n_images)), zoom=(1*math.pow(2,j-1)), scaleeach=false}
  end

  n_images = math.min(testData.size(), 196)  -- 12 x 12
  for j=1,num_hpf_banks do
    im = {
      data = testData.data[j][{{1,n_images}, {}, {}, {}}],
      labels = testData.labels[{{1,n_images}, {}}]
    }
    im.data = im.data:double()
    im.labels = im.labels:double()
    if (j == 1 and visualize_data_labels == 1) then
      for i=1,n_images do
        max_val = im.data[{i,{},{},{}}]:max()
        for k=1,num_coeff-1,num_coeff_per_feature do
          uv_pos = {
            round(bank_dim[j][1] * im.labels[{i, k}]), 
            round(bank_dim[j][2] * im.labels[{i, k+1}])
          }
          -- Add vert line at feature
          for v=(uv_pos[2]-1),(uv_pos[2]+1) do
            im.data[{i,1,v,uv_pos[1]}] = max_val
          end
          -- Add horiz line at feature
          for u=(uv_pos[1]-1),(uv_pos[1]+1) do
            im.data[{i,1,uv_pos[2],u}] = max_val
          end
        end
      end
    end
    image.display{image=im.data, padding=2, nrow=math.floor(math.sqrt(n_images)), zoom=(1*math.pow(2,j-1)), scaleeach=false}
  end
  im = nil
end
