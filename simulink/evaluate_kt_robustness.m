function [attitude,robust] = evaluate_kt_robustness(folder,archiveTraces)
% Run 70 initial poses and 384 robustness cases and export report figures.
arguments
    folder (1,1) string = fullfile(fileparts(mfilename('fullpath')),"outputs_robustness")
    archiveTraces (1,1) logical = true
end
if ~isfolder(folder)
    mkdir(folder)
end
[ok,attributes] = fileattrib(folder);
assert(ok,'Output folder is inaccessible.')
folder = string(attributes.Name);
traceFolder = "";
if archiveTraces
    traceFolder = string(tempname);
    mkdir(traceFolder)
    removeTraces = onCleanup(@() rmdir(traceFolder,'s')); %#ok<NASGU>
end

check_kt_robustness();
model = 'KT_Robot_Robustness';
disabled = onCleanup(@() set_param(model,'FastRestart','off')); %#ok<NASGU>
set_param(model,'FastRestart','on')
methods = ["baseline","curvature"];
k = 0;
for y = [-.9 -.6 -.3 0 .3 .6 .9]
    for angle = [-90 -45 0 45 90]
        for method = methods
            k = k+1;
            [data,attitudeRows(k,1)] = run_kt_robustness("corners",method, ...
                "ideal",1,[0;y;deg2rad(angle)]); %#ok<AGROW>
            assertPose(data,[0;y;deg2rad(angle)])
            saveTrace(traceFolder,sprintf('pose_%02d.csv',k),data)
        end
    end
end
attitude = struct2table(attitudeRows);
attitude.trace_id = compose("pose_%02d", (1:height(attitude)).');
writetable(attitude,fullfile(folder,'initial_pose_trials.csv'))
fprintf('Initial pose sweep complete: %d runs.\n',height(attitude))

k = 0;
profiles = ["ideal","delay","outage","motor","radius","slip", ...
    "paper_noise","combined_medium","combined_strong"];
for scenario = ["corners","s_curve"]
    set_param(model,'FastRestart','off')
    set_param(model,'FastRestart','on')
    for profile = profiles
        seeds = 1;
        if ismember(profile,profiles(7:end))
            seeds = 1:30;
        end
        for seed = seeds
            for method = methods
                k = k+1;
                [data,robustRows(k,1)] = run_kt_robustness(scenario,method,profile,seed); %#ok<AGROW>
                assertPose(data,[0;-.6;0])
                saveTrace(traceFolder,sprintf('robust_%03d.csv',k),data)
                if scenario=="corners" && profile=="combined_strong" && seed==1
                    writetable(data,fullfile(folder,"incident_"+method+".csv"))
                end
            end
        end
        fprintf('%s / %s complete (%d of 384).\n',scenario,profile,k)
    end
end
set_param(model,'FastRestart','off')
robust = struct2table(robustRows);
robust.trace_id = compose("robust_%03d", (1:height(robust)).');
assert(height(attitude)==70 && height(robust)==384,'Unexpected run count.')
nominal = attitude(abs(attitude.y0_m+.6)<1e-10 & attitude.theta0_deg==0,:);
for i = 1:height(nominal)
    correct = robust(robust.method==nominal.method(i) & robust.scenario=="corners" & robust.profile=="ideal",:);
    assert(abs(nominal.rmse_distance_m(i)-correct.rmse_distance_m)<1e-10 && ...
        nominal.decision_time_s(i)==correct.decision_time_s)
end
writetable(robust,fullfile(folder,'robustness_trials.csv'))
if archiveTraces
    zip(fullfile(folder,'simulation_traces.zip'),'*.csv',traceFolder)
end
summarize_kt_robustness(attitude,robust,folder)
simulinkVersion = ver('simulink');
manifest = struct('matlab_release',version('-release'),'simulink_version', ...
    simulinkVersion.Version,'dt_s',.05,'lookahead_m',.55,'control_time_s',120, ...
    'max_settling_s',2,'seed_generator','mt19937ar','seeds',1:30, ...
    'initial_pose_runs',height(attitude),'robustness_runs',height(robust));
writelines(jsonencode(manifest,'PrettyPrint',true),fullfile(folder,'experiment_manifest.json'))
plot_kt_robustness(attitude,robust,folder)
end

function assertPose(data,pose)
assert(max(abs(data{1,2:4}.'-pose))<1e-12,'Initial pose did not update in Fast Restart.')
end

function saveTrace(folder,name,data)
if strlength(folder)>0
    writetable(data,fullfile(folder,name))
end
end




