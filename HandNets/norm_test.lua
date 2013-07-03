require 'nn'
require 'image'
require 'torch'
require 'xlua'    -- xlua provides useful tools, like progress bars
require 'optim'   -- an optimization package, for online and batch methods
torch. setnumthreads(4)

width = 11
height = 12
torch.manualSeed(123)
n_states = 3
im = torch.DoubleTensor(n_states, height, width)
-- val = 1
-- for i=1,n_states do
--   for v=1,height do
--     for u=1,width do
--       im[{i,v,u}] = i * (u+(height-v))
--     end
--   end
-- end
im:apply(function()
  return torch.uniform()
  end)

print(im)

kernelsize = 7  -- must be odd
kernelrad = (kernelsize-1)/2
kernel = image.gaussian1D(kernelsize)
model = nn.Sequential()
model:add(nn.SpatialSubtractiveNormalization(n_states, kernel))
subnorm =  model:get(1)

output = model:forward(im)

print("subnorm.output")
print(subnorm.output)
print("subnorm.localsums")
print(subnorm.localsums)
print("subnorm.adjustedsums")
print(subnorm.adjustedsums)
print("subnorm.coef")
print(subnorm.coef)

-- Try a manual kernel (to see if we can get the functionality correct on
-- the edge cases)
kernel_prime = kernel:div(kernel:sum())  -- normalize kernel (done once)
-- Create adjustment coeff by integrating an image of ones
coef = torch.DoubleTensor(height, width)
for v=1,height do
  for u=1,width do
    coef[{v,u}] = 0
    for v_filt=-kernelrad,kernelrad do
      for u_filt=-kernelrad,kernelrad do
        u_in = u + u_filt
        v_in = v + v_filt
        if (u_in >= 1 and u_in <= width and
            v_in >= 1 and v_in <= height) then 
          coef[{v,u}] = coef[{v,u}] + kernel_prime[{u_filt + kernelrad + 1}] * kernel_prime[{v_filt + kernelrad + 1}] * 1
        end
      end
    end
    coef[{v,u}] = coef[{v,u}] / n_states
  end
end
print("coef should be the same as subnorm.coef")
print(coef)

out_man = torch.DoubleTensor(n_states, height, width)
-- Perform filtering first --> WITH ZERO PADDING FOR EDGE CASES! - HORIZONTAL
for i=1,n_states do
  for v=1,height do
    for u=1,width do
      out_man[{i,v,u}] = 0
      for u_filt=-kernelrad,kernelrad do
        u_in = u + u_filt
        if (u_in >= 1 and u_in <= width) then 
          out_man[{i,v,u}] = out_man[{i,v,u}] + 
            kernel_prime[{u_filt + kernelrad + 1}] * im[{i, v, u_in}]
        end
      end
    end
  end
end
out_man2 = torch.DoubleTensor(n_states, height, width)
-- Perform filtering first --> WITH ZERO PADDING FOR EDGE CASES! - VERTICAL
for i=1,n_states do
  for v=1,height do
    for u=1,width do
      out_man2[{i,v,u}] = 0
      for v_filt=-kernelrad,kernelrad do
        v_in = v + v_filt
        if (v_in >= 1 and v_in <= height) then 
          out_man2[{i,v,u}] = out_man2[{i,v,u}] + 
            kernel_prime[{v_filt + kernelrad + 1}] * out_man[{i, v_in, u}]
        end
      end
    end
  end
end
print("out_man2 should be the same as subnorm.localsums")
print(out_man2)

-- Now average accross states
constants = torch.DoubleTensor(height, width)
for v=1,height do
  for u=1,width do
    constants[{v,u}] = 0
    for i=1,n_states do
      constants[{v,u}] = constants[{v,u}] + out_man2[{i,v,u}]
    end
    constants[{v,u}] = (constants[{v,u}] / n_states) / n_states
  end
end
print("constants should be the same as subnorm.localsums")
print(constants)

-- Now create adjusted sums
constants_adj = torch.DoubleTensor(height, width)
for v=1,height do
  for u=1,width do
    constants_adj[{v,u}] = constants[{v,u}] / coef[{v,u}]
  end
end
print("constants_adj should be the same as subnorm.adjustedsums")
print(constants_adj)

-- Now subtract off the normalization factor
for i=1,n_states do
  for v=1,height do
    for u=1,width do
      out_man[{i,v,u}] = im[{i, v, u}] - constants_adj[{v,u}]
    end
  end
end

print("out_man should be the same as subnorm.output")
print(out_man)

delta = 0;
for i=1,n_states do
  for v=1,height do
    for u=1,width do
      delta = delta + math.abs(out_man[{i,v,u}] - output[{i,v,u}])
    end
  end
end

print("l1norm(out_man - output) =      <-- should be very small")
print(delta)