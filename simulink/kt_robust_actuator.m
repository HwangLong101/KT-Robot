function [realWheels,body,slip] = kt_robust_actuator(command,robot,impact,event,time,dt)
% First-order wheel response plus radius mismatch and local translational slip.
persistent previous
if isempty(previous)
    previous = zeros(2,1);
end
realWheels = command;
if impact(6) > 0
    decay = exp(-dt/impact(6));
    realWheels = decay*previous + (1-decay)*command;
end
previous = realWheels;
rim = robot(1)*(1+impact(7:8)).*realWheels;
slip = zeros(2,1);
if event(3) ~= 0
    elapsed = round((time-event(1))/dt)*dt;
    slip = impact(9)*[.03*sin(1.2*elapsed+.045408*pi); ...
                      .05*cos(1.2*elapsed-.00459*pi)];
end
body = [(rim(1)+rim(2))/2 + slip(1); slip(2); (rim(2)-rim(1))/robot(2)];
end

