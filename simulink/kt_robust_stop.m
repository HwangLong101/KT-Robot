function stop = kt_robust_stop(terminal,wheels,event,time,dt)
% End when settled, or after two seconds even if the plant is still moving.
settled = max(abs(wheels)) <= .1 && event(3) == 0;
tailExpired = round((time-terminal(2))/dt)>=round(2/dt);
stop = terminal(2)>=0 && (settled || tailExpired);
end

