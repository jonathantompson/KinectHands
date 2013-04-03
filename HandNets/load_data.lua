
-- ******* Some preliminary calculations *********
w = width
h = height
bank_dim = {}
data_file_size = 0
for i=1,num_hpf_banks do
  table.insert(bank_dim, {h, w})
  data_file_size = data_file_size + w * h
  w = w / 2
  h = h / 2
end
w = nil  -- To avoid confusion
h = nil

-- ************ Get Data locations ***************
print '==> Scanning directory for hand data...'
if paths.dirp(im_dir) == false then
  print("Couldn't find image directory")
  return
else 
  -- Collect the filenames
  files = {}
  i = 1
  for f in paths.files(im_dir) do
    files[i] = f
    i = i+1
  end
  -- Note: the files are in random order
end
-- Partition files into their respective groups
-- coeffl_files = {}
coeffr_files = {}
hpf_depth_files = {}
depth_files = {}
for i=1,#files,1 do
  if files[i] ~= nil then
    if string.find(files[i], "coeffr_hands_") ~= nil then
      table.insert(coeffr_files, files[i])
--    elseif string.find(files[i], "coeffl_hands_") ~= nil then
--      table.insert(coeffl_files, files[i])
    elseif string.find(files[i], "hpf_processed_hands_") ~= nil then
      table.insert(hpf_depth_files, files[i])
    elseif string.find(files[i], "processed_hands_") ~= nil then
      table.insert(depth_files, files[i])
    end
  end
end 

if paths.dirp(test_im_dir) == false then
  print("Couldn't find test image directory")
  return
else 
  -- Collect the filenames
  files = {}
  i = 1
  for f in paths.files(test_im_dir) do
    files[i] = f
    i = i+1
  end
  -- Note: The files are in random order
end
-- Partition files into their respective groups
-- coeffl_files = {}
test_coeffr_files = {}
test_hpf_depth_files = {}
test_depth_files = {}
for i=1,#files,1 do
  if files[i] ~= nil then
    if string.find(files[i], "coeffr_hands_") ~= nil then
      table.insert(test_coeffr_files, files[i])
--    elseif string.find(files[i], "coeffl_hands_") ~= nil then
--      table.insert(coeffl_files, files[i])
    elseif string.find(files[i], "hpf_processed_hands_") ~= nil then
      table.insert(test_hpf_depth_files, files[i])
    elseif string.find(files[i], "processed_hands_") ~= nil then
      table.insert(test_depth_files, files[i])
    end
  end
end 

if (use_hpf_depth == 1) then
  im_files = hpf_depth_files
  test_im_files = test_hpf_depth_files
else
  im_files = depth_files
  test_im_files = test_depth_files
end

print("==> Sorting files so indicies match...")
function stringComp (a, b) 
  return string.lower(a) < string.lower(b) 
end
table.sort(im_files, stringComp)
table.sort(test_im_files, stringComp)
table.sort(coeffr_files, stringComp)
table.sort(test_coeffr_files, stringComp)

-- ************ Randomly permute the files ***********
print("==> Permuting the files...")
shuffle_files_right(coeffr_files, im_files)
shuffle_files_right(test_coeffr_files, test_im_files)

-- ************ Check the file order ***********
print("==> Verifying the file order...")
for i=1,#im_files do
  if (string.sub(im_files[i],17,-5) ~= string.sub(coeffr_files[i],14,-5)) then
    print("Image file number doesn't match coeff file number!")
    return
  end
end
for i=1,#test_im_files do
  if (string.sub(test_im_files[i],17,-5) ~= string.sub(test_coeffr_files[i],14,-5)) then
    print("Image file number doesn't match coeff file number!")
    return
  end
end

-- ************ Load data from Disk ***************
print '==> Loading hand data from directory...'
nfiles = math.floor(#im_files / frame_stride)
tesize = math.floor(nfiles / test_data_rate) + 1
trsize = nfiles - tesize
tesize = tesize + #test_im_files  -- Also add in the training files from the separate dir
if tesize <= 0 then
  print("test set size is <= 0")
  return
end
trainData = {
  files = {},
  data = {},  -- Multi-bank input (usually 3)
  labels = torch.FloatTensor(trsize, num_coeff),
  size = function() return trsize end
}
testData = {
  files = {},
  data = {},
  labels = torch.FloatTensor(tesize, num_coeff),
  size = function() return tesize end
}

for i=1,(num_hpf_banks-skip_banks) do
  table.insert(trainData.data, torch.FloatTensor(trsize, 1, 
    bank_dim[i+skip_banks][1], bank_dim[i+skip_banks][2]))
  table.insert(testData.data, torch.FloatTensor(tesize, 1, 
    bank_dim[i+skip_banks][1], bank_dim[i+skip_banks][2]))
end

-- LOAD IN TRAINING SET DIRECTORY --> SOME IMAGES GO TO TEST SET AS WELL!
itr = 1
ite = 1
for i=1,nfiles do
  -- disp progress
  progress(i, nfiles)

  -- Read in the sample
  coeff_file = torch.DiskFile(im_dir .. coeffr_files[i*frame_stride], 'r')
  coeff_file:binary()
  coeff_data = coeff_file:readFloat(num_coeff)
  coeff_file:close()

  hpf_depth_file = torch.DiskFile(im_dir .. im_files[i*frame_stride],'r')
  hpf_depth_file:binary()
  hpf_depth_data = hpf_depth_file:readFloat(data_file_size)
  hpf_depth_file:close()

  if math.mod(i, test_data_rate) ~= 0 then
    if itr <= trsize then
      -- this sample is training data
      -- We need to split the long vector
      ind = 1
      for j=1,num_hpf_banks do
        if (j > skip_banks) then
          trainData.data[j-skip_banks][{itr, 1, {}, {}}] = torch.FloatTensor(
            hpf_depth_data, ind, torch.LongStorage{bank_dim[j][1], 
            bank_dim[j][2]}):float()
        end
        ind = ind + (bank_dim[j][1]*bank_dim[j][2]) -- Move pointer forward
      end
      trainData.labels[{itr, {}}] = torch.FloatTensor(coeff_data, 1,
        torch.LongStorage{num_coeff}):float()
      trainData.files[itr] = im_files[i]
      itr = itr + 1
    end
  else
    if ite <= tesize then
      -- this sample is test data
      -- We need to split the long vector
      ind = 1
      for j=1,num_hpf_banks do
        if (j > skip_banks) then
          testData.data[j-skip_banks][{ite, 1, {}, {}}] = torch.FloatTensor(
            hpf_depth_data, ind, torch.LongStorage{bank_dim[j][1], 
            bank_dim[j][2]})
        end
        ind = ind + (bank_dim[j][1]*bank_dim[j][2]) -- Move pointer forward
      end
      testData.labels[{ite, {}}] = torch.FloatTensor(coeff_data, 1,
        torch.LongStorage{num_coeff}):float()
      testData.files[ite] = im_files[i*frame_stride]
      ite = ite + 1
    end
  end
end

-- NOW LOAD IN TEST SET DIRECTORY
for i=1,#test_im_files do
  -- disp progress
  progress(i, #test_im_files)

  -- Read in the sample
  coeff_file = torch.DiskFile(test_im_dir .. test_coeffr_files[i], 'r')
  coeff_file:binary()
  coeff_data = coeff_file:readFloat(num_coeff)
  coeff_file:close()

  hpf_depth_file = torch.DiskFile(test_im_dir .. test_im_files[i],'r')
  hpf_depth_file:binary()
  hpf_depth_data = hpf_depth_file:readFloat(data_file_size)
  hpf_depth_file:close()
  if ite <= tesize then
    -- this sample is test data
    -- We need to split the long vector
    ind = 1
    for j=1,num_hpf_banks do
      if (j > skip_banks) then
        testData.data[j - skip_banks][{ite, 1, {}, {}}] = torch.FloatTensor(
          hpf_depth_data, ind, torch.LongStorage{bank_dim[j][1], 
          bank_dim[j][2]})
      end
      ind = ind + (bank_dim[j][1]*bank_dim[j][2]) -- Move pointer forward
    end
    testData.labels[{ite, {}}] = torch.FloatTensor(coeff_data, 1,
      torch.LongStorage{num_coeff}):float()
    testData.files[ite] = test_im_files[i]
    ite = ite + 1
  end
end

----- Hack: something goes wrong for certain cases, just redefine what we loaded
tesize = ite - 1
trsize = itr - 1
nfiles = tesize + trsize
for i=trsize+1,#trainData.files do
  table.remove(trainData.files, trsize+1)
end
for j=1,num_hpf_banks-skip_banks do
  trainData.data[j] = trainData.data[j][{{1,trsize}, {}, {}, {}}]
end
trainData.labels = trainData.labels[{{1,trsize}, {}}]
trainData.size = function() return trsize end
for i=tesize+1,#testData.files do
  table.remove(testData.files, tesize+1)
end
for j=1,num_hpf_banks-skip_banks do
  testData.data[j] = testData.data[j][{{1,tesize}, {}, {}, {}}]
end
testData.labels = testData.labels[{{1,tesize}, {}}]
testData.size = function() return tesize end

print(string.format("    Loaded %d test set images and %d training set images", 
  tesize, trsize))

-- Force a return (just load data)
-- if (true) then
--   return
-- end

-- ********************** Converting data to cuda **********************
print '==> Converting data to cudaTensor'
-- for j=1,testData.size() do
--   for k=1,num_hpf_banks do
--    testData.data[j][k] = testData.data[j][k]:cuda()
--   end
-- end
-- for j=1,trainData.size() do
--   for k=1,num_hpf_banks do
--     trainData.data[j][k] = trainData.data[j][k]:cuda()
--   end
-- end
if (loss ~= 0) then
  trainData.labels = trainData.labels:cuda()
  testData.labels = testData.labels:cuda()
end
