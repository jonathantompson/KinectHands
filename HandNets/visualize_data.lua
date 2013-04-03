if (visualize_data == 1) then
  n_images = math.min(trainData.size(), 256)
  for j=1,num_hpf_banks do
    im = {
      data = trainData.data[j][{{1,n_images}, {}, {}, {}}]
    }
    im.data = im.data:double()
    image.display{image=im.data, padding=2, nrow=math.floor(math.sqrt(n_images)), zoom=(0.75*math.pow(2,j-1)), scaleeach=false}
  end
  -- image.display(trainData.data[1][{1,1,{},{}}])

  n_images = math.min(testData.size(), 256)
  for j=1,num_hpf_banks do
    im = {
      data = testData.data[j][{{1,n_images}, {}, {}, {}}]
    }
    im.data = im.data:double()
    image.display{image=im.data, padding=2, nrow=math.floor(math.sqrt(n_images)), zoom=(0.75*math.pow(2,j-1)), scaleeach=false}
  end
  -- image.display(testData.data[{1,1,{},{}}])
  im = nil
end
