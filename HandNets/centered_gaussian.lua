indices_image_x = nil
indices_image_y = nil
indices_image_x_local = nil
indices_image_y_local = nil

function image.centered_gaussian(...)
   -- process args
   local _, size, sigma, amplitude, normalize, 
   width, height, sigma_horz, sigma_vert, center_x, center_y = dok.unpack(
      {...},
      'image.gaussian',
      'returns a 2D gaussian kernel',
      {arg='size', type='number', help='kernel size (size x size)', default=3},
      {arg='sigma', type='number', help='sigma (horizontal and vertical)', default=0.25},
      {arg='amplitude', type='number', help='amplitute of the gaussian (max value)', default=1},
      {arg='normalize', type='number', help='normalize kernel (exc Amplitude)', default=false},
      {arg='width', type='number', help='kernel width', defaulta='size'},
      {arg='height', type='number', help='kernel height', defaulta='size'},
      {arg='sigma_horz', type='number', help='horizontal sigma', defaulta='sigma'},
      {arg='sigma_vert', type='number', help='vertical sigma', defaulta='sigma'},
      {arg='center_x', type='number', help='X gauss center [0 to 1]', defaulta=nil},
      {arg='center_y', type='number', help='Y gauss center [0 to 1]', defaulta=nil}
   )
   
   -- local vars
   if (center_x == nil) then
     center_x = width/2
   else
     center_x = center_x * width
   end
   if (center_y == nil) then
     center_y = height/2
   else
     center_y = center_y * height
   end
   
   -- Create global data structures that will make multiple calls faster
   if (indices_image_x == nil or indices_image_x:size()[2] ~= width) then
     indices_image_x = torch.FloatTensor(height, width)
     indices_image_x_local = torch.FloatTensor(height, width)
     for i=1,height do
       for j=1,width do
         indices_image_x[i][j] = j - 0.5
       end
     end
   end
   if (indices_image_y == nil or indices_image_y:size()[1] ~= height) then
     indices_image_y = torch.FloatTensor(height, width)
     indices_image_y_local = torch.FloatTensor(height, width)
     for i=1,height do
       for j=1,width do
         indices_image_y[i][j] = i - 0.5
       end
     end
   end

   indices_image_x_local:copy(indices_image_x)
   indices_image_y_local:copy(indices_image_y)

   indices_image_x_local:add((-center_x))
   indices_image_x_local:mul(1.0 / (sigma_horz*width))
   indices_image_x_local:pow(2)
   indices_image_x_local:mul(0.5)

   indices_image_y_local:add((-center_y))
   indices_image_y_local:mul(1.0 / (sigma_vert*height))
   indices_image_y_local:pow(2)
   indices_image_y_local:mul(0.5)

   local gauss = indices_image_x_local + indices_image_y_local
   gauss:mul(-1)
   gauss:exp()
   gauss:mul(amplitude)
   
   -- The above is suppose to replicate this:
   --[[
   for i=1,height do
      for j=1,width do
         gauss[i][j] = amplitude * math.exp(-(math.pow((j-center_x) / (sigma_horz*width),2)/2 
                                            + math.pow((i-center_y) / (sigma_vert*height),2)/2))
      end
   end
   --]]

   if normalize then
      gauss:div(gauss:sum())
   end
   return gauss
end






