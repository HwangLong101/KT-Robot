function run_kt_checks()
% Analytical motion and closed-loop edge cases beyond the 42-run matrix.
closeTo = @(a,b) assert(max(abs(a-b),[],'all') < 1e-12);
closeTo(pp_pose([1;2;0],[0.2;0],2),[1.4;2;0])
closeTo(pp_pose([1;2;0],[0;1],1),[1;2;1])
closeTo(pp_pose([0;0;0],[0.5;1],pi/2),[0.5;0.5;pi/2])
cfg = pp_config();
[twist,wheels] = pp_wheels([1;4],cfg.robot);
closeTo(twist(2)/twist(1),4)
assert(max(abs(wheels))<=12+1e-12 && abs(twist(2))<=2+1e-12)

[data,metrics] = run_kt_robot("straight","baseline",.55,.05,120,[0;-.6;pi]);
assert(metrics.reached_goal && any(data.mode_code==1),'Heading recovery failed.')
path = [0 0;2 0;2 2;0 2;0 0];
[data,metrics] = run_kt_robot(path,"baseline",.55,.05,120,[0;0;0]);
assert(metrics.reached_goal && metrics.time_s>10 && data.progress_m(end)>7.6, ...
    'Closed path stopped before reaching its final segment.')
[data,metrics] = run_kt_robot("straight","baseline",.55,.05,.1);
assert(~metrics.reached_goal && data.mode_code(end)==3 && height(data)==3, ...
    'Timeout did not produce the expected terminal sample.')
disp('Analytical motion, wheel limits, heading recovery, closed path and timeout passed.')
end

