function shuffle_data(t)
  local n = t:size(1)
 
  while n >= 2 do
    -- n is now the last pertinent index
    local k = math.random(n) -- 1 <= k <= n
    -- swap n and k
    local temp = t[{k,{},{},{}}]
    t[{k,{},{},{}}] = t[{n,{},{},{}}]
    t[{n,{},{},{}}] = temp
    n = n - 1
  end
 
  return t
end