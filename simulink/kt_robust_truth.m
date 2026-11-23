function [geometry,event] = kt_robust_truth(pose,path,time,impact,dt)
% Ground-truth path geometry and the first threshold crossing.
persistent eventStep
if isempty(eventStep)
    eventStep = -1;
end
xy = pose(1:2);
progress = 0;
error = inf;
for i = 1:size(path,1)-1
    p = path(i,1:2).';
    d = path(i+1,1:2).' - p;
    length = path(i,4);
    fraction = min(max(dot(xy-p,d)/length^2,0),1);
    gap = norm(p+fraction*d-xy);
    if gap < error
        progress = path(i,3) + fraction*length;
        error = gap;
    end
end
goalDistance = norm(path(end,1:2).' - xy);
step = round(time/dt);
if eventStep < 0 && progress > impact(10)
    eventStep = step;
end
event = [-1;0;0];
if eventStep >= 0
    elapsed = (step-eventStep)*dt;
    event = [eventStep*dt; double(elapsed < impact(5)); ...
        double(impact(9) ~= 0 && elapsed < 3)];
end
geometry = [progress;error;goalDistance];
end

