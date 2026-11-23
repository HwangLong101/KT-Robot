function plot_kt_results(folder)
% Generate publication figures using the actual MATLAB and Python sample logs.
root = fileparts(mfilename('fullpath'));
if nargin == 0
    folder = fullfile(root,'outputs');
end
methods = ["baseline","curvature"];
labels = ["Cơ sở","Giảm tốc theo độ cong"];
palette = [0.10 0.36 0.64; 0.78 0.30 0.10];
ids = "corners_" + methods + "_L0.55_dt0.050";
for i = 1:2
    ref = fullfile(root,'..','pure_pursuit_evaluation','outputs','runs',ids(i));
    python{i} = readtable(fullfile(ref,'trajectory.csv')); %#ok<AGROW>
    sims{i} = readtable(fullfile(folder,ids(i) + "_simulink.csv")); %#ok<AGROW>
end
path = readmatrix(fullfile(ref,'reference.csv'));
positions = cumsum(hypot(diff(path(:,1)),diff(path(:,2))));
corners = positions(1:end-1);
for i = 1:2
    data = sims{i};
    for j = 1:numel(corners)
        samples = abs(data.progress_m-corners(j))<=.55;
        row = table(methods(i),corners(j),max(data.path_error_m(samples)), ...
            max(abs(data.left_rad_s(samples))),max(abs(data.right_rad_s(samples))), ...
            'VariableNames',{'method','corner_m','max_path_error_m', ...
            'max_left_rad_s','max_right_rad_s'});
        if i==1 && j==1
            local = row;
        else
            local = [local; row]; %#ok<AGROW>
        end
    end
end
writetable(local,fullfile(folder,'corner_metrics.csv'))

fig = newFigure([100 100 900 640]);
layout = tiledlayout(fig,2,2,'TileSpacing','compact','Padding','compact');
for i = 1:2
    a = nexttile(layout);
    p = python{i}; s = sims{i};
    plot(a,path(:,1),path(:,2),'-','Color',[.58 .58 .58],'LineWidth',1.5); hold(a,'on')
    plot(a,p.x_m,p.y_m,'-','Color',palette(1,:),'LineWidth',1.7)
    plot(a,s.x_m,s.y_m,'--','Color',palette(2,:),'LineWidth',1.3)
    axis(a,'equal'); xlim(a,[-.5 8.6]); ylim(a,[-.9 6.4]); grid(a,'on')
    xlabel(a,'x (m)'); ylabel(a,'y (m)'); title(a,labels(i))
    if i == 1
        legend(a,{'Tham chiếu','Python','Simulink'},'Location','northwest')
    end
end
a = nexttile(layout,[1 2]); hold(a,'on')
for i = 1:2
    p = python{i}; s = sims{i};
    plot(a,s.time_s,hypot(s.x_m-p.x_m,s.y_m-p.y_m), ...
        'Color',palette(i,:),'LineWidth',1.3)
end
xlabel(a,'Thời gian (s)'); ylabel(a,'Sai khác vị trí (m)'); grid(a,'on')
legend(a,labels,'Location','northwest'); xlim(a,[0 max(sims{2}.time_s)])
saveFigure(fig,folder,'matlab_python_simulink');

fig = newFigure([100 100 900 780]);
layout = tiledlayout(fig,3,1,'TileSpacing','compact','Padding','compact');
a = nexttile(layout); hold(a,'on')
for i = 1:2
    s = sims{i};
    plot(a,s.progress_m,s.path_error_m,'Color',palette(i,:),'LineWidth',1.5)
end
legend(a,labels,'Location','northeast')
ylabel(a,'Sai số bám (m)'); title(a,'Sai số bám đường')
progressAxis(a,corners,positions(end))
for i = 1:2
    a = nexttile(layout); hold(a,'on')
    s = sims{i};
    plot(a,s.progress_m,s.left_rad_s,'Color',palette(1,:),'LineWidth',1.5)
    plot(a,s.progress_m,s.right_rad_s,'--','Color',palette(2,:),'LineWidth',1.5)
    yline(a,12,'k:','HandleVisibility','off')
    yline(a,-12,'k:','HandleVisibility','off')
    ylim(a,[-13 14]); ylabel(a,'Tốc độ (rad/s)'); title(a,labels(i))
    if i == 1
        legend(a,{'Bánh trái','Bánh phải'},'Location','southeast')
    end
    progressAxis(a,corners,positions(end))
end
xlabel(a,'Tiến độ dọc tuyến (m)')
saveFigure(fig,folder,'matlab_corner_tracking');
end

function fig = newFigure(position)
fig = figure('Visible','off','Color','white','Position',position);
set(fig,'DefaultAxesFontName','Arial','DefaultAxesFontSize',12, ...
    'DefaultTextFontName','Arial','DefaultTextInterpreter','none', ...
    'DefaultLegendInterpreter','none')
end

function progressAxis(a,corners,endPosition)
for corner = corners.'
    xline(a,corner,'--','Color',[.60 .60 .60],'HandleVisibility','off')
end
grid(a,'on'); xlim(a,[0 endPosition])
end

function saveFigure(fig,folder,name)
exportgraphics(fig,fullfile(folder,[name '.png']),'Resolution',320)
exportgraphics(fig,fullfile(folder,[name '.pdf']),'ContentType','vector')
close(fig)
end


