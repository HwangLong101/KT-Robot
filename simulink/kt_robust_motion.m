function nextPose = kt_robust_motion(pose,body,dt)
% Integrate constant body-frame longitudinal and lateral speeds for a sample.
theta = pose(3);
v = body(1);
vy = body(2);
omega = body(3);
nextTheta = theta + omega*dt;
if abs(omega) < 1e-10
    displacement = v*[cos(theta);sin(theta)]*dt ...
        + vy*[-sin(theta);cos(theta)]*dt;
else
    ds = sin(nextTheta)-sin(theta);
    dc = cos(theta)-cos(nextTheta);
    displacement = (v/omega)*[ds;dc] + (vy/omega)*[-dc;ds];
end
nextPose = [pose(1:2) + displacement; mod(nextTheta+pi,2*pi)-pi];
end

