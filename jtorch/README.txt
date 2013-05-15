This is a CPP framework for doing the forward prop of various torch nodes.

The library consists of a simple lua codebase for recursively saving a torch model (all in the ./lua folder):

-- TORCH USAGE:
dofile("./lua/save_nn_node.lua")
model = nn.Sequential()
model.add(x)
saveNNNode(model, "my_model.bin")

As well as a CPP framework for loading it and doing the forward prop.  See jtorch_test for more details of usage.

It uses OpenCL for all GPU computing.  The following stages have full implementations:

SpatialConvolution
SpatialConvolutionCUDA  --> Only 3D tensor is supported (no batch processing)
SpatialConvolutionMap   --> Full implementation but it is slow (all on CPU)
Sequential
Parallel  --> Uses the c++ jtorch::Table as a container for multiple jtorch::Tensor<float> instances
Tanh
Threshold
Linear
SpatialLPPooling  --> Full implementation but it is slow (all on CPU)
SpatialMaxPooling
SpatialMaxPoolingCUDA  --> Only 3D tensor is supported (no batch processing)
SpatialSubtractiveNormalization
SpatialDivisiveNormalization
SpatialContrastiveNormalization

The following stages have partial implementations:
JoinTable  --> Nothing fancy.  Just concatenates along the 0th dimension.
Reshape  --> Just a pass through stage
Transpose  --> Just a pass through stage