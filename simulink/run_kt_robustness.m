function [data,metrics] = run_kt_robustness(scenario,method,profile,seed,initialPose)
% Simulate one disturbance configuration and evaluate against the true pose.
arguments
    scenario (1,1) string = "corners"
    method (1,1) string = "baseline"
    profile (1,1) string = "combined_strong"
    seed (1,1) double {mustBeInteger,mustBeNonnegative} = 1
    initialPose (3,1) double {mustBeFinite} = [0;-0.6;0]
end
cfg = kt_robust_config(scenario,method,profile,seed,initialPose);
model = 'KT_Robot_Robustness';
root = fileparts(mfilename('fullpath'));
if ~bdIsLoaded(model)
    load_system(fullfile(root,[model '.slx']))
end
input = Simulink.SimulationInput(model);
keys = fieldnames(cfg);
for i = 1:numel(keys)
    input = input.setVariable(keys{i},cfg.(keys{i}),'Workspace',model);
end
out = sim(input);
raw = out.robust_log;
if ndims(raw) == 3
    raw = reshape(raw,[],size(raw,3)).';
end
names = {'time_s','x_m','y_m','theta_rad','measured_x_m','measured_y_m', ...
    'measured_theta_rad','command_left_rad_s','command_right_rad_s','left_rad_s', ...
    'right_rad_s','v_m_s','vy_m_s','omega_rad_s','true_progress_m','path_error_m', ...
    'goal_distance_m','event_time_s','outage','slip_active','mode_code', ...
    'decision_time_s','slip_x_m_s','slip_y_m_s','measured_path_error_m', ...
    'measured_goal_distance_m','progress_m','target_x_m','target_y_m','controller_mode'};
data = array2table(raw,'VariableNames',names);

assert(all(isfinite(raw),'all'),'Simulation log contains non-finite signals.')
mode = data.mode_code(end);
decided = data.decision_time_s(end);
firstDecision = find(data.decision_time_s>=0,1);
assert(ismember(mode,[2 3]) && ~isempty(firstDecision),'Missing terminal controller decision.')
assert(all(raw(firstDecision:end,8:9)==0,'all'),'Commands must be zero after the decision.')
assert(data.time_s(end)-decided <= 2+1e-10,'Tail exceeded two seconds.')
wheelMax = max(abs(raw(:,10:11)),[],'all');
commandMax = max(abs(raw(:,8:9)),[],'all');
assert(wheelMax<=12+1e-10 && commandMax<=12+1e-10,'Wheel speed exceeds 12 rad/s.')
settled = max(abs(raw(end,10:11))) <= .1;
reached = mode == 2 && data.goal_distance_m(end)<=.05 && settled;
status = "success";
if mode == 3
    status = "timeout";
elseif ~reached
    status = "false_goal";
end
distance = hypot(data.v_m_s(1:end-1),data.vy_m_s(1:end-1))*cfg.Ts;
e2 = data.path_error_m.^2;
goalTime = NaN;
if mode == 2
    goalTime = decided;
end
jointSpeed = abs(data.v_m_s.*data.omega_rad_s);
metrics = struct('scenario',scenario,'method',method,'profile',profile,'seed',seed, ...
    'x0_m',initialPose(1),'y0_m',initialPose(2),'theta0_deg',rad2deg(initialPose(3)), ...
    'status',status,'goal_announced',mode==2,'settled',settled,'decision_time_s',decided, ...
    'goal_announcement_s',goalTime,'end_time_s',data.time_s(end), ...
    'final_goal_error_m',data.goal_distance_m(end),'goal_error_at_decision_m', ...
    data.goal_distance_m(firstDecision),'final_wheel_rad_s',max(abs(raw(end,10:11))), ...
    'rmse_distance_m',sqrt(sum(distance.*(e2(1:end-1)+e2(2:end))/2)/sum(distance)), ...
    'alignment_s',sum(data.mode_code==1)*cfg.Ts,'max_execution_v_omega',max(jointSpeed), ...
    'max_command_wheel_rad_s',commandMax,'max_actual_wheel_rad_s',wheelMax, ...
    'event_time_s',data.event_time_s(end), ...
    'outage_samples',sum(data.outage),'slip_samples',sum(data.slip_active));
end

