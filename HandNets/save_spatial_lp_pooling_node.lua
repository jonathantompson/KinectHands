function saveSpatialLPPoolingNode(node, ofile)

  -- The layout is as follows:
  -- 1. filter width (int)
  -- 2. filter height (int)
  -- 3. filter input/output features (int)
  -- 4. pnorm (either 1 or 2) (int)

  ofile:writeInt(node.kW)
  ofile:writeInt(node.kH)
  ofile:writeInt(node.nInputPlane)

  if (torch.typename(node:get(1)) == "nn.Square")
    pnorm = 2
  elseif (torch.typename(node:get(1)) == "nn.Power")
    pnorm = node:get(1).pow
  else
    error("saveSpatialLPPoolingNode() - ERROR: Cannot determine pnorm")
    return
  end
  ofile:writeInt(pnorm)

end
