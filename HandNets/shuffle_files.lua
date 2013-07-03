function shuffle_files(coeffr, coeffl, im)
  local n = #im
 
  while n >= 2 do
    -- n is now the last pertinent index
    local k = math.random(n) -- 1 <= k <= n
    -- swap n and k
    -- image
    local temp = im[k]
    im[k] = im[n]
    im[n] = temp
    -- coeffl
    temp = coeffl[k]
    coeffl[k] = coeffl[n]
    coeffl[n] = temp
    -- coeffr
    temp = coeffr[k]
    coeffr[k] = coeffr[n]
    coeffr[n] = temp
    n = n - 1
  end
end

function shuffle_files_right(coeffr, im)
  local n = #im
 
  while n >= 2 do
    -- n is now the last pertinent index
    local k = math.random(n) -- 1 <= k <= n
    -- swap n and k
    -- image
    local temp = im[k]
    im[k] = im[n]
    im[n] = temp
    -- coeffr
    temp = coeffr[k]
    coeffr[k] = coeffr[n]
    coeffr[n] = temp
    n = n - 1
  end
end
