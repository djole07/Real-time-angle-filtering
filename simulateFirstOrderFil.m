function [angleFil] = simulateFirstOrderFil(angleAcc, Ts, Tf)
p = exp(-Ts / Tf);
angleFil = zeros(1, length(angleAcc));
angleFil(1) = angleAcc(1);

for i=2:length(angleAcc)
    angleFil(i) = p*angleFil(i-1) + (1-p)*angleAcc(i);
end

end

