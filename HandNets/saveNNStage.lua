function saveNNStage(nn_stage, nonlinear, out_file)
  -- The layout is as follows:
  -- 1. filter input features (int)
  -- 2. filter output features (int)
  -- 3. filter weights (float array)
  -- 4. Non-linearity enum type (int)
  local noutputs = nn_stage.weight:size()[1]
  local ninputs = nn_stage.weight:size()[2]
  out_file:writeInt(noutputs)
  out_file:writeInt(ninputs)
 
  for i=1,noutputs do
    for v=1,ninputs do
      out_file:writeFloat(nn_stage.weight[{i, v}])
    end
  end

  if (nonlinear == "Tanh") then
    out_file:writeInt(0)
  elseif (nonlinear == "None") then
    out_file:writeInt(1)
  else
    out_file:writeInt(2)
  end
end