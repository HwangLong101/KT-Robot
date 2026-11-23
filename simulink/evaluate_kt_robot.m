function summary = evaluate_kt_robot(outputFolder)
% Reproduce the 42 Python configurations in Simulink and match samples by index.
root = fileparts(mfilename('fullpath'));
if nargin == 0
    outputFolder = fullfile(root,'outputs');
end
if ~isfolder(outputFolder)
    mkdir(outputFolder)
end
pythonFolder = fullfile(root,'..','pure_pursuit_evaluation','outputs');
trials = readtable(fullfile(pythonFolder,'all_trials.csv'),'TextType','string');
assert(height(trials)==42 && numel(unique(trials.run_id))==42,'Expected 42 distinct Python trials.')
tolerance = 1e-7;
for i = 1:height(trials)
    row = trials(i,:);
    refFolder = fullfile(pythonFolder,'runs',row.run_id);
    reference = readtable(fullfile(refFolder,'trajectory.csv'));
    points = readmatrix(fullfile(refFolder,'reference.csv'));
    [data,metrics] = run_kt_robot(points,row.method,row.lookahead_m,row.dt_s);
    assert(isequal(data.Properties.VariableNames,reference.Properties.VariableNames), ...
        'Python and Simulink CSV columns differ.')
    result = compare(row,reference,data,metrics,tolerance);
    results(i,1) = result; %#ok<AGROW>
    if row.lookahead_m == .55 && row.dt_s == .05
        writetable(data,fullfile(outputFolder,row.run_id + "_simulink.csv"))
    end
    fprintf('%2d/%d %-39s %d samples  max delta %.3g  %s\n',i,height(trials), ...
        row.run_id,height(data),result.max_channel_delta,result.failed_signals)
end
summary = struct2table(results);
writetable(summary,fullfile(outputFolder,'python_simulink_comparison.csv'));
nominal = summary(summary.lookahead_m==.55 & summary.dt_s==.05,:);
writetable(nominal,fullfile(outputFolder,'nominal_comparison.csv'));
assert(all(summary.pass),'Some trials differ. See python_simulink_comparison.csv.')
run_kt_checks();
plot_kt_results(outputFolder);
end

function result = compare(row,reference,data,metrics,tolerance)
% Use common rows for diagnostic deltas even when sample counts differ.
n = min(height(reference),height(data));
p = reference{1:n,:};
s = data{1:n,:};
delta = abs(s-p);
names = string(data.Properties.VariableNames);
maxDelta = max(delta(:,1:end-1),[],1);
failed = names([maxDelta>tolerance, any(delta(:,end)~=0)]);
if height(data) ~= height(reference)
    failed(end+1) = "sample_count";
end
if data.mode_code(end) ~= reference.mode_code(end)
    failed(end+1) = "terminal_mode";
end
if abs(data.time_s(end)-reference.time_s(end)) > tolerance
    failed(end+1) = "terminal_time";
end
if ~metrics.reached_goal || data.goal_distance_m(end)>.05
    failed(end+1) = "goal";
end
if metrics.max_abs_wheel_speed_rad_s>12+1e-10 || metrics.max_abs_yaw_rate_rad_s>2+1e-10
    failed(end+1) = "actuator_limit";
end
if row.method=="curvature" && metrics.max_lateral_demand_m_s2>.12+1e-10
    failed(end+1) = "lateral_limit";
end
result = struct('run_id',row.run_id,'scenario',row.scenario,'method',row.method, ...
    'lookahead_m',row.lookahead_m,'dt_s',row.dt_s, ...
    'samples_python',height(reference),'samples_simulink',height(data), ...
    'goal_python',reference.mode_code(end)==2,'goal_simulink',metrics.reached_goal, ...
    'time_python_s',reference.time_s(end),'time_simulink_s',metrics.time_s, ...
    'rmse_s_python_m',row.distance_weighted_rmse_m, ...
    'rmse_s_simulink_m',metrics.distance_weighted_rmse_m, ...
    'rmse_s_delta_m',abs(row.distance_weighted_rmse_m-metrics.distance_weighted_rmse_m), ...
    'max_position_delta_m',max(hypot(delta(:,2),delta(:,3))), ...
    'max_wheel_delta_rad_s',max(delta(:,7:8),[],'all'), ...
    'max_channel_delta',max(maxDelta), ...
    'final_goal_error_m',metrics.final_goal_error_m, ...
    'path_rmse_m',metrics.path_rmse_m,'path_max_error_m',metrics.path_max_error_m, ...
    'max_lateral_demand_m_s2',metrics.max_lateral_demand_m_s2, ...
    'max_lateral_demand_after_1m_m_s2',metrics.max_lateral_demand_after_1m_m_s2, ...
    'max_abs_wheel_speed_rad_s',metrics.max_abs_wheel_speed_rad_s, ...
    'max_abs_yaw_rate_rad_s',metrics.max_abs_yaw_rate_rad_s, ...
    'pass',isempty(failed),'failed_signals',strjoin(failed,"|"));
for j = 1:numel(maxDelta)
    result.("max_delta_" + names(j)) = maxDelta(j);
end
end

