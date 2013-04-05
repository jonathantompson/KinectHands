function saveSpatialSubtractiveNormalizationNode(node, ofile)

  if (node.kernel:dim() > 1) then
    error("saveSpatialSubtractiveNormalizationNode() - ERROR: Only 1D kernels are supported for now!")
    return
  end

  -- The layout is as follows:
  -- 1. filter kernel size (int)
  -- 2. filter kernel (float array)
  -- 3. filter input/output features (int)

  ofile:writeInt(node.kernel:size())
  for i=1,node.kernel:size() do
    ofile:writeFloat(node.kernel[{i}])
  end
  ofile:writeInt(node.nInputPlane)
end
