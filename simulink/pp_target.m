function [target, progress, error, goalDistance, remaining] = pp_target(pose, previous, path, settings)
% Find the first forward circle intersection on the reference polyline.
xy = pose(1:2);
n = size(path,1);
total = path(n,3);
lookahead = settings(1);
progress = project(xy, path, previous, min(total, previous + max(2*lookahead,1)));
[~, error] = project(xy, path, 0, total);
goal = path(n,1:2).';
goalDistance = norm(goal - xy);
remaining = total - progress;
target = goal;
if remaining <= lookahead
    return
end

for i = 1:n-1
    if path(i+1,3) <= progress
        continue
    end
    p = path(i,1:2).';
    d = path(i+1,1:2).' - p;
    length = path(i,4);
    q = p - xy;
    a = dot(d,d);
    b = 2*dot(q,d);
    c = dot(q,q) - lookahead^2;
    disc = b*b - 4*a*c;
    if disc < 0
        continue
    end
    root = sqrt(disc);
    for u = [(-b-root)/(2*a), (-b+root)/(2*a)]
        if u >= 0 && u <= 1 && path(i,3) + u*length > progress + 1e-8
            target = p + u*d;
            return
        end
    end
end

s = min(progress + lookahead,total);
for i = 1:n-1
    if s < path(i+1,3) || i == n-1
        p = path(i,1:2).';
        d = path(i+1,1:2).' - p;
        target = p + ((s-path(i,3))/path(i,4))*d;
        return
    end
end
end

function [s, distance] = project(xy, path, first, last)
% Strict improvement preserves the first segment in a projection tie.
s = first;
distance = inf;
for i = 1:size(path,1)-1
    if path(i+1,3) < first || path(i,3) > last
        continue
    end
    p = path(i,1:2).';
    d = path(i+1,1:2).' - p;
    length = path(i,4);
    u = dot(xy-p,d)/length^2;
    low = max(0,(first-path(i,3))/length);
    high = min(1,(last-path(i,3))/length);
    u = min(max(u,low),high);
    e = norm(p + u*d - xy);
    if e < distance
        distance = e;
        s = path(i,3) + u*length;
    end
end
end

