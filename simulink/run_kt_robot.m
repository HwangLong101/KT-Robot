function [data,metrics] = run_kt_robot(scenario,method,lookahead,dt,maxTime,initialPose)
% Run the native Simulink model and return a table with Python's log schema.
arguments
    scenario = "corners"
    method = "baseline"
    lookahead = 0.55
    dt = 0.05
    maxTime = 120
    initialPose (3,1) double {mustBeFinite} = [0;-0.6;0]
end
cfg = pp_config(scenario,method,lookahead,dt,maxTime);
cfg.initialPose = initialPose;
model = 'KT_Robot_Pure_Pursuit';
root = fileparts(mfilename('fullpath'));
if ~bdIsLoaded(model)
    load_system(fullfile(root,[model '.slx']));
end
input = Simulink.SimulationInput(model);
keys = fieldnames(cfg);
for i = 1:numel(keys)
    input = input.setVariable(keys{i},cfg.(keys{i}),'Workspace',model);
end
out = sim(input);
raw = out.sample_log;
if ndims(raw) == 3
    raw = reshape(raw,[],size(raw,3)).';
end
labels = {'time_s','x_m','y_m','theta_rad','v_m_s','omega_rad_s', ...
    'left_rad_s','right_rad_s','path_error_m','goal_distance_m','progress_m', ...
    'target_x_m','target_y_m','mode_code'};
data = array2table(raw,'VariableNames',labels);

distance = abs(data.v_m_s(1:end-1))*cfg.Ts;
errorSquared = data.path_error_m.^2;
squaredPerStep = (errorSquared(1:end-1)+errorSquared(2:end))/2;
lateral = abs(data.v_m_s.*data.omega_rad_s);
metrics = struct('reached_goal',data.mode_code(end)==2,'sample_count',height(data), ...
    'time_s',data.time_s(end),'final_goal_error_m',data.goal_distance_m(end), ...
    'path_rmse_m',sqrt(mean(errorSquared)), ...
    'path_max_error_m',max(data.path_error_m), ...
    'distance_weighted_rmse_m',sqrt(sum(distance.*squaredPerStep)/sum(distance)), ...
    'max_lateral_demand_m_s2',max(lateral), ...
    'max_lateral_demand_after_1m_m_s2',max([NaN; lateral(data.progress_m>=1)],[],'omitnan'), ...
    'max_abs_wheel_speed_rad_s',max(abs(raw(:,7:8)),[],'all'), ...
    'max_abs_yaw_rate_rad_s',max(abs(data.omega_rad_s)));
assert(data.mode_code(end)>=2 && all(raw(end,5:8)==0), ...
    'The model must record a terminal sample with zero applied commands.')
assert(all(isfinite(raw),'all'),'Simulation log contains non-finite results.')
end


