function [command,terminal] = kt_robust_command(wheels,tracking,time)
% Latch the first terminal controller decision and hold zero thereafter.
persistent decision decidedAt
if isempty(decision)
    decision = 0;
    decidedAt = -1;
end
mode = tracking(6);
if decision == 0 && mode >= 2
    decision = mode;
    decidedAt = time;
end
command = [wheels(1); wheels(2)];
if decision ~= 0
    command = zeros(2,1);
    mode = decision;
end
terminal = [mode;decidedAt];
end


