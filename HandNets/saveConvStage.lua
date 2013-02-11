function saveConvStage(conv_stage, norm, poolsize, pooltype, nonlinear, out_file)
  -- The layout is as follows:
  -- 1. filter width (int)
  -- 2. filter height (int)
  -- 3. filter input features (int)
  -- 4. filter output features (int)
  -- 5. filter fan in (int)
  -- 6. filter weights (float array)
  -- 7. filter connection table (short array)
  -- 8. Normalization enum type (int)
  -- 9. Pooling enum type (int)
  -- 10. Pooling size (int)
  -- 11. Non-linearity enum type (int)
  -- 12. Biases (float)

  out_file:writeInt(conv_stage.kW)
  out_file:writeInt(conv_stage.kH)
  out_file:writeInt(conv_stage.nInputPlane)
  out_file:writeInt(conv_stage.nOutputPlane)
  local fanin = conv_stage.connTableRev:size()[2]
  out_file:writeInt(fanin)
 
  for i=1,(conv_stage.nOutputPlane * fanin) do
    for v=1,conv_stage.kH do
      for u=1,conv_stage.kW do
        out_file:writeFloat(conv_stage.weight[{i, v, u}])
      end
    end
  end

  for i=1,conv_stage.nOutputPlane do
    for v=1,fanin do
      out_file:writeShort(conv_stage.connTableRev[{i, v, 1}] - 1)
      out_file:writeShort(conv_stage.connTableRev[{i, v, 2}] - 1)
    end
  end

  if (norm == "spac sub.") then
    out_file:writeInt(0)
  else
    out_file:writeInt(1)
  end

  if (pooltype == math.huge) then
    out_file:writeInt(2)
  else
    out_file:writeInt(pooltype)
  end
  
  out_file:writeInt(poolsize)

  if (nonlinear == "Tanh") then
    out_file:writeInt(0)
  elseif (nonlinear == "SoftShrink") then
    out_file:writeInt(1)
  else 
    out_file:writeInt(2)
  end

  for i=1,(conv_stage.nOutputPlane) do
    out_file:writeFloat(conv_stage.bias[{i}])
  end
end