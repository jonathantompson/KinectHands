local sigmoid_cc, parent = torch.class('nn.sigmoid_cc', 'nn.Module')

 function sigmoid_cc:__init() 
    parent.__init(self)  
 end 

 function sigmoid_cc:updateOutput(input)
    self.output = input:clone():apply(function(x)  
                               local sig = 1/(1+math.exp(-x))
                               return sig*(1-sig)  
                              end)
    return self.output 
 end

 function sigmoid_cc:updateGradInput(input, gradOutput)
    self.gradInput = input:clone():apply(function(x) 
                                 local sig = 1/(1+math.exp(-x))
                                 return (sig*(1-sig)^2 - sig^2*(1-sig))^2 
                                 end)
    self.gradInput:cmul(gradOutput)
    return self.gradInput 
  end

local shrink_cc, parent = torch.class('nn.shrink_cc', 'nn.Module')

 function shrink_cc:__init(lam) 
    parent.__init(self) 
    self.lambda = lam or 0.5 
 end 

 function shrink_cc:updateOutput(input)
    self.output = input:clone():apply(function(x)  
                              if x <= -self.lambda then 
                                  return -x-self.lambda  
                              elseif x >= self.lambda then 
                                  return  x-self.lambda 
                              else 
                                  return 0 
                              end
                              end)
    return self.output 
 end

 function shrink_cc:updateGradInput(input, gradOutput)
    self.gradInput = input:clone():apply(function(x) 
                                 if x <= -self.lambda then 
                                     return -1 
                                 elseif x >= self.lambda then
                                     return  1
                                 else
                                     return  0
                                 end
                                 end)
    self.gradInput:cmul(gradOutput)
    return self.gradInput 
  end

local shrink0, parent = torch.class('nn.shrink0', 'nn.Module')

 function shrink0:__init(lam) 
    parent.__init(self) 
    self.lambda = lam or 0.5 
 end 

 function shrink0:updateOutput(input)
    self.output = input:clone():apply(function(x)  
                              if x <= -self.lambda then 
                                  return x 
                              elseif x >= self.lambda then 
                                  return x
                              else
                                  return 0 
                              end
                              end)
    return self.output 
 end

 function shrink0:updateGradInput(input, gradOutput)
    self.gradInput = input:clone():apply(function(x) 
                              if x <= -self.lambda then 
                                  return 1
                              elseif x >= self.lambda then 
                                  return 1 
                              else 
                                  return 0
                              end
                              end)
    self.gradInput:cmul(gradOutput)
    return self.gradInput 
  end
                         

local sat_linear, parent = torch.class('nn.sat_linear', 'nn.Module')

 function sat_linear:__init(lam)
    parent.__init(self) 
    self.lambda = lam or 0.5 
 end 

 function sat_linear:updateOutput(input)
    self.output = input:clone():apply(function(x)  
                              if x <= -self.lambda then 
                                  return -self.lambda  
                              elseif x>= self.lambda then 
                                  return self.lambda
                              else
                                  return x
                              end
                              end)
    return self.output 
 end

 function sat_linear:updateGradInput(input, gradOutput)
    self.gradInput = input:clone():apply(function(x) 
                                 if x <= -self.lambda then 
                                    return 0
                                 elseif x >= self.lambda then 
                                     return 0 
                                 else
                                     return 1
                                 end
                                 end)
    self.gradInput:cmul(gradOutput)
    return self.gradInput 
  end

local lim_linear, parent = torch.class('nn.lim_linear', 'nn.Module')

 function lim_linear:__init(lam)
    parent.__init(self) 
    self.lambda = lam or 0.5 
 end 

 function lim_linear:updateOutput(input)
    self.output = input:clone():apply(function(x)  
                              if x <= -self.lambda then 
                                  return 0
                              elseif x>= self.lambda then 
                                  return 0
                              else
                                  return x
                              end
                              end)
    return self.output 
 end

 function lim_linear:updateGradInput(input, gradOutput)
    self.gradInput = input:clone():apply(function(x) 
                                 if x <= -self.lambda then 
                                    return 0
                                 elseif x >= self.lambda then 
                                     return 0 
                                 else
                                     return 1
                                 end
                                 end)
    self.gradInput:cmul(gradOutput)
    return self.gradInput 
  end


local sat_linear_cc, parent = torch.class('nn.sat_linear_cc', 'nn.Module')

 function sat_linear_cc:__init(lam) 
    parent.__init(self) 
    self.lambda = lam or 0.5 
 end 

 function sat_linear_cc:updateOutput(input)
    self.output = input:clone():apply(function(x)  
                              if x <= -self.lambda or x >= self.lambda then 
                                  return 0  
                              elseif x > -self.lambda and x < 0 then 
                                  return self.lambda + x 
                              elseif x > 0 and x < self.lambda then 
                                  return self.lambda - x 
                              end
                              end)
    return self.output 
 end

 function sat_linear_cc:updateGradInput(input, gradOutput)
    self.gradInput = input:clone():apply(function(x) 
                              if x <= -self.lambda or x >= self.lambda then 
                                  return 0  
                              elseif x > -self.lambda and x < 0 then 
                                  return 1 
                              elseif x > 0 and x < self.lambda then 
                                  return -1  
                              end
                                 end)
    self.gradInput:cmul(gradOutput)
    return self.gradInput 
  end

local sat_linear01, parent = torch.class('nn.sat_linear01', 'nn.Module')

 function sat_linear01:__init(lam) 
    parent.__init(self) 
    self.lambda = lam or 0.5 
 end 

function sat_linear01:updateOutput(input)
    self.output = input:clone():apply(function(x)  
                              if x <= 0 then 
                                  return 0  
                              elseif x >= self.lambda then 
                                  return self.lambda
                              else
                                  return x
                              end
                              end)
    return self.output 
 end

 function sat_linear01:updateGradInput(input, gradOutput)
    self.gradInput = input:clone():apply(function(x) 
                                 if x <= 0 then 
                                    return 0
                                 elseif x >= self.lambda then 
                                     return 0 
                                 else
                                     return 1
                                 end
                                 end)
    self.gradInput:cmul(gradOutput)
    return self.gradInput 
  end

local sat_linear01_cc, parent = torch.class('nn.sat_linear01_cc', 'nn.Module')

 function sat_linear01_cc:__init(lam) 
    parent.__init(self) 
    self.lambda = lam or 0.5 
 end 

 function sat_linear01_cc:updateOutput(input)
    self.output = input:clone():apply(function(x)  
                              if x <= 0 or x >= self.lambda then 
                                  return 0  
                              elseif x > 0 and x < self.lambda/2 then 
                                  return x 
                              elseif x >= self.lambda/2 and x < self.lambda then 
                                  return self.lambda - x 
                              end
                              end)
    return self.output 
 end

 function sat_linear01_cc:updateGradInput(input, gradOutput)
    self.gradInput = input:clone():apply(function(x) 
                              if x <= 0 or x >= self.lambda then 
                                  return 0  
                              elseif x > 0 and x < self.lambda/2 then 
                                  return 1 
                              elseif x >= self.lambda/2 and x < self.lambda then 
                                  return -1  
                              end
                                 end)
    self.gradInput:cmul(gradOutput)
    return self.gradInput 
  end

local ramp, parent = torch.class('nn.ramp', 'nn.Module')

 function ramp:__init(lam) 
    parent.__init(self) 
    self.lambda = lam or 0.5 
 end 

 function ramp:updateOutput(input)
    self.output = input:clone():apply(function(x)  
                              if x <= self.lambda then 
                                  return 0  
                              else 
                                  return  x-self.lambda 
                              end
                              end)
    return self.output 
 end

 function ramp:updateGradInput(input, gradOutput)
    self.gradInput = input:clone():apply(function(x) 
                                 if x <= self.lambda then 
                                     return 0 
                                 else
                                     return 1
                                 end
                                 end)
    self.gradInput:cmul(gradOutput)
    return self.gradInput 
  end

local neg_ramp, parent = torch.class('nn.neg_ramp', 'nn.Module')

 function neg_ramp:__init(lam) 
    parent.__init(self) 
    self.lambda = lam or 0.5 
 end 

 function neg_ramp:updateOutput(input)
    self.output = input:clone():apply(function(x)  
                              if x <= self.lambda then 
                                  return -0.5  
                              else 
                                  return  x-self.lambda 
                              end
                              end)
    return self.output 
 end

 function neg_ramp:updateGradInput(input, gradOutput)
    self.gradInput = input:clone():apply(function(x) 
                                 if x <= self.lambda then 
                                     return 0 
                                 else
                                     return 1
                                 end
                                 end)
    self.gradInput:cmul(gradOutput)
    return self.gradInput 
  end

local mul_const, parent = torch.class('nn.mul_const', 'nn.Module')

 function mul_const:__init(const) 
    parent.__init(self) 
    self.const = const or 1
 end 

 function mul_const:updateOutput(input)
    self.output:resize(input:size())
    self.output:copy(input) 
    self.output:mul(self.const) 
    return self.output 
 end

 function mul_const:updateGradInput(input, gradOutput)
    self.gradInput:resize(gradOutput:size()):fill(self.const)  
    self.gradInput:cmul(gradOutput) 
    return self.gradInput 
  end
