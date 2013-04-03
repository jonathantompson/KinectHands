print '==> defining loss function'
-- print '   using ABS criterion'
-- criterion = nn.AbsCriterion()
-- criterion.sizeAverage = false
print '    using MSE criterion'
criterion = nn.MSECriterion()  
criterion:cuda()
-- print(criterion)
