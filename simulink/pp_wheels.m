function [twist,wheels] = pp_wheels(request, robot)
% Limit yaw first, then both wheels together to retain curvature.
radius = robot(1);
track = robot(2);
limited = request/max(1,abs(request(2))/robot(4));
v = limited(1);
omega = limited(2);
left = (v-track*omega/2)/radius;
right = (v+track*omega/2)/radius;
wheels = [left;right]/max([1,abs(left)/robot(3),abs(right)/robot(3)]);
twist = [radius*(wheels(1)+wheels(2))/2; radius*(wheels(2)-wheels(1))/track];
end

