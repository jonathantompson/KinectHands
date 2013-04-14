dofile("distort.lua")

function rotateData(im, labels)
  local min_max_rotation_deg = 30
  
  local rotation_angle
  im_distort, rotation_angle = distort(im, 0.0, 0.0, 1.0, min_max_rotation_deg)
  print(rotation_angle)

  return im_distort, labels
end


