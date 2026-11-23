function measured = kt_robust_sensor(pose,event,time,dt,impact,noise)
% Add Gaussian noise, delay by whole samples, and hold during outages.
persistent history lastValid
if isempty(history)
    history = zeros(3,4);
    lastValid = zeros(3,1);
end
if round(time/dt) == 0
    history(:,:) = repmat(pose,1,4);
    lastValid = pose;
end
sample = pose + impact(1:3).*noise(:,min(round(time/dt)+1,size(noise,2)));
delay = impact(4);
delivered = sample;
if delay > 0
    delivered = history(:,delay);
end
history = [sample history(:,1:3)];
if event(2) == 0
    lastValid = delivered;
end
measured = lastValid;
end

