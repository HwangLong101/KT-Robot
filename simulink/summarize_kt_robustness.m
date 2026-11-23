function summarize_kt_robustness(attitude,robust,folder)
% Summaries use true pose. Goal times use true successes only.
methods = ["baseline","curvature"];
keys = unique(robust(:,{'scenario','profile','method'}),'rows','stable');
for k = 1:height(keys)
    group = robust(robust.scenario==keys.scenario(k) & ...
        robust.profile==keys.profile(k) & robust.method==keys.method(k),:);
    groups(k,1) = summary(group); %#ok<AGROW>
end
writetable([keys struct2table(groups)],fullfile(folder,'robustness_summary.csv'))

aggregateKeys = unique(robust(:,{'profile','method'}),'rows','stable');
for k = 1:height(aggregateKeys)
    group = robust(robust.profile==aggregateKeys.profile(k) & ...
        robust.method==aggregateKeys.method(k),:);
    aggregate(k,1) = summary(group); %#ok<AGROW>
end
writetable([aggregateKeys struct2table(aggregate)],fullfile(folder,'robustness_aggregate.csv'))

for k = 1:2
    group = attitude(attitude.method==methods(k),:);
    poseRows(k,1) = summary(group); %#ok<AGROW>
end
writetable([table(methods.','VariableNames',{'method'}) struct2table(poseRows)], ...
    fullfile(folder,'initial_pose_summary.csv'))

keys = unique(robust(:,{'scenario','profile'}),'rows','stable');
pairs = cell(height(keys),1);
paired = cell(height(keys),1);
for k = 1:height(keys)
    group = robust(robust.scenario==keys.scenario(k) & robust.profile==keys.profile(k),:);
    a = sortrows(group(group.method=="baseline",:),'seed');
    b = sortrows(group(group.method=="curvature",:),'seed');
    assert(isequal(a.seed,b.seed),'Missing paired seed.')
    both = a.status=="success" & b.status=="success";
    timeDelta = b.goal_announcement_s-a.goal_announcement_s;
    timeDelta(~both) = NaN;
    n = height(a);
    delta = table(repmat(keys.scenario(k),n,1),repmat(keys.profile(k),n,1),a.seed, ...
        a.status,b.status,both,timeDelta,b.rmse_distance_m-a.rmse_distance_m, ...
        b.final_goal_error_m-a.final_goal_error_m,b.alignment_s-a.alignment_s, ...
        b.max_execution_v_omega-a.max_execution_v_omega, ...
        'VariableNames',{'scenario','profile','seed','baseline_status','curvature_status', ...
        'both_success','delta_goal_s','delta_rmse_m','delta_final_m','delta_alignment_s','delta_v_omega'});
    pairs{k} = delta;
    stats = struct('pairs',n,'both_success',sum(both));
    for name = ["delta_goal_s","delta_rmse_m","delta_final_m","delta_alignment_s","delta_v_omega"]
        stats = quantiles(stats,delta.(name),name);
    end
    paired{k} = [keys(k,:) struct2table(stats)];
end
writetable(vertcat(pairs{:}),fullfile(folder,'robustness_pairs.csv'))
writetable(vertcat(paired{:}),fullfile(folder,'robustness_paired_summary.csv'))
end

function s = summary(t)
success = t.status=="success";
s = struct('runs',height(t),'successful',sum(success),'timeouts',sum(t.status=="timeout"), ...
    'false_goals',sum(t.status=="false_goal"),'alignment_runs',sum(t.alignment_s>0), ...
    'max_goal_s',maxOrNaN(t.goal_announcement_s(success)), ...
    'max_success_rmse_m',maxOrNaN(t.rmse_distance_m(success)));
s = quantiles(s,t.goal_announcement_s(success),"goal_s");
s = quantiles(s,t.rmse_distance_m,"rmse_m");
s = quantiles(s,t.rmse_distance_m(success),"success_rmse_m");
s = quantiles(s,t.final_goal_error_m,"final_m");
s = quantiles(s,t.alignment_s,"alignment_s");
s = quantiles(s,t.max_execution_v_omega,"v_omega");
end

function s = quantiles(s,values,name)
values = values(isfinite(values));
q = [NaN NaN NaN];
if ~isempty(values)
    q = prctile(values,[25 50 75]);
end
s.(name+"_q1") = q(1);
s.(name+"_median") = q(2);
s.(name+"_q3") = q(3);
end

function value = maxOrNaN(values)
value = max([NaN;values],[],'omitnan');
end


