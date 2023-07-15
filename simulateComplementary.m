function [angleCompl] = simulateComplementary(angleAcc, angleGyro, tau)
angleCompl = zeros(1, length(angleAcc));
gAngle = zeros(1, length(angleAcc));

angleCompl(1) = angleGyro(1);
gAngle = angleGyro(1);

for i=2:length(angleAcc)
    rate = angleGyro(i) - angleGyro(i-1);
    gAngle = gAngle + rate;
    gAngle = tau*gAngle + (1-tau)*angleAcc(i);
    angleCompl(i) = gAngle;
end

end

