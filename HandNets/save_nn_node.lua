dofile('save_sequential_node.lua')
dofile('save_parallel_node.lua')
dofile('save_tanh_node.lua')
dofile('save_threshold_node.lua')
dofile('save_linear_node.lua')
dofile('save_reshape_node.lua')
dofile('save_spatial_convolution_node.lua')
dofile('save_spatial_convolution_map_node.lua')
dofile('save_spatial_lp_pooling_node.lua')
dofile('save_spatial_max_pooling_node.lua')
dofile('save_spatial_subtractive_normalization_node.lua')
dofile('save_spatial_divisive_normalization_node.lua')
dofile('save_spatial_contrastive_normalization_node.lua')
dofile('save_join_table_node.lua')

function saveNNNode(node, ofile)
  -- Just send the node off to the correct routine depending on it's type
  -- Note that the type enum must match 
  -- kinect_interface/include/kinect_interface/hand_net/torch_stage.h
  class_str = torch.typename(node)
  print("saving " .. class_str .. "...")
  if (class_str == "nn.Sequential") then
     ofile:writeInt(1)
     saveSequentialNode(node, ofile)
  elseif (class_str == "nn.ParallelTable") then
     ofile:writeInt(2)
     saveParallelNode(node, ofile)
  elseif (class_str == "nn.Tanh") then
     ofile:writeInt(3)
     saveTanhNode(node, ofile)
  elseif (class_str == "nn.Threshold") then
     ofile:writeInt(4)
     saveThresholdNode(node, ofile)
  elseif (class_str == "nn.Linear") then
     ofile:writeInt(5)
     saveLinearNode(node, ofile)
  elseif (class_str == "nn.Reshape") then
     ofile:writeInt(6)
     saveReshapeNode(node, ofile)
  elseif (class_str == "nn.SpatialConvolution") then
     -- Note: SpatialConvolution gets saved with same index 
     --       as SpatialConvolutionMap
     ofile:writeInt(7)
     saveSpatialConvolutionNode(node, ofile)
  elseif (class_str == "nn.SpatialConvolutionMap") then
     ofile:writeInt(7)
     saveSpatialConvolutionMapNode(node, ofile)
  elseif (class_str == "nn.SpatialLPPooling") then
     ofile:writeInt(8)
     saveSpatialLPPoolingNode(node, ofile)
  elseif (class_str == "nn.SpatialMaxPooling") then
     ofile:writeInt(9)
     saveSpatialMaxPoolingNode(node, ofile)
  elseif (class_str == "nn.SpatialSubtractiveNormalization") then
     ofile:writeInt(10)
     saveSpatialSubtractiveNormalizationNode(node, ofile)
  elseif (class_str == "nn.SpatialDivisiveNormalization") then
     ofile:writeInt(11)
     saveSpatialDivisiveNormalizationNode(node, ofile)
  elseif (class_str == "nn.SpatialContrastiveNormalization") then
     ofile:writeInt(12)
     saveSpatialContrastiveNormalizationNode(node, ofile)
  elseif (class_str == "nn.JoinTable") then
     ofile:writeInt(13)
     saveJoinTableNode(node, ofile)
  else
     error('Node type ' .. class_str .. ' is not recognized.')
     return
  end
end

