function saveSpatialConvolutionCUDANode(node, ofile)
  -- Note: SpatialConvolution gets saved as if it was a 
  --       SpatialConvolutionMap, we just fill out a full
  --       connection table.

  -- The layout is as follows:
  -- 1. filter width (int)
  -- 2. filter height (int)
  -- 3. filter input features (int)
  -- 4. filter output features (int)
  -- 5. filter fan in (int)
  -- 6. filter weights (float array)
  -- 7. filter connection table (short array)
  -- 8. filter Biases (float)

  ofile:writeInt(node.kW)
  ofile:writeInt(node.kH)
  ofile:writeInt(node.nInputPlane)
  ofile:writeInt(node.nOutputPlane)

  local fanin = node.nInputPlane
  ofile:writeInt(fanin)

  for i=1,(node.nOutputPlane) do
    for j=1,(node.nInputPlane) do
      for v=1,node.kH do
        for u=1,node.kW do
          -- The only difference is the reordering of weights
          ofile:writeFloat(node.weight[{j, v, u, i}])
        end
      end
    end
  end

  local cur_mat = 0;
  for i=1,node.nOutputPlane do
    for j=1,(node.nInputPlane) do
      ofile:writeShort(j - 1)  -- input feature
      ofile:writeShort(cur_mat)  -- weight matrix
      cur_mat = cur_mat + 1
    end
  end

  for i=1,(node.nOutputPlane) do
    ofile:writeFloat(node.bias[{i}])
  end

end
