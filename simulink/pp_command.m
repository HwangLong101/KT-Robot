function [request, mode, stop] = pp_command(pose, target, goalDistance, remaining, time, settings)
% Modes: pursuit (0), alignment (1), goal (2), and timeout (3).
lookahead = settings(1);
nearEnd = remaining <= lookahead;
reached = nearEnd && goalDistance <= settings(3);
timeout = round(time/settings(9)) >= settings(10);
stop = reached || timeout;
request = zeros(2,1);
if stop
    mode = 3;
    if reached
        mode = 2;
    end
    return
end

offset = target - pose(1:2);
theta = pose(3);
alpha = mod(atan2(offset(2),offset(1)) - theta + pi,2*pi) - pi;
if abs(alpha) > settings(5)
    mode = 1;
    request(2) = settings(6)*alpha;
    return
end

mode = 0;
v = settings(2);
if nearEnd
    v = min(v,settings(4)*goalDistance);
end
yBody = -sin(theta)*offset(1) + cos(theta)*offset(2);
kappa = 2*yBody/max(dot(offset,offset),1e-12);
if settings(7) == 1
    v = min(v,sqrt(settings(8)/max(abs(kappa),1e-12)));
end
request = [v; v*kappa];
end

