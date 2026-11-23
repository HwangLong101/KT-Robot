function nextPose = pp_pose(pose,twist,dt)
% Exact motion under a zero-order-held command over one sample.
theta = pose(3);
v = twist(1);
omega = twist(2);
nextTheta = theta + omega*dt;
if abs(omega) < 1e-10
    displacement = v*[cos(theta);sin(theta)]*dt;
else
    displacement = (v/omega)*[sin(nextTheta)-sin(theta);cos(theta)-cos(nextTheta)];
end
nextPose = [pose(1:2) + displacement; mod(nextTheta+pi,2*pi)-pi];
end

