function trim(s)
  return s:find'^%s*$' and '' or s:match'^%s*(.*%S)'
end