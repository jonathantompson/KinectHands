if (visualize_model == 1) then
  for j=1,num_hpf_banks do
    n_images = math.min(256, model:get(1):get(j):get(1).weight:size()[1])
    im = {
      data = model:get(1):get(j):get(1).weight[{{1,n_images}, {}, {}, {}}]
    }
    im.data = im.data:double()
    image.display{image=im.data, padding=2, nrow=math.floor(math.sqrt(n_images)), zoom=(10), scaleeach=false}
  end

  for j=1,num_hpf_banks do
    n_images = math.min(256, model:get(1):get(j):get(4).weight:size()[1])
    im = {
      data = model:get(1):get(j):get(4).weight[{{1,n_images}, {1}, {}, {}}]
    }
    im.data = im.data:double()
    image.display{image=im.data, padding=2, nrow=math.floor(math.sqrt(n_images)), zoom=(10), scaleeach=false}
  end

  im = nil

  print '==> here is the model:'
  print(model)
end

