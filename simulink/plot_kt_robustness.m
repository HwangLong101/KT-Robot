function plot_kt_robustness(attitude,robust,folder)
% Draw all report plots from the sampled Simulink results.
methods = ["baseline","curvature"];
titles = ["Cơ sở","Giảm tốc theo độ cong"];
colors = [0.10 .36 .64; .78 .30 .10];
yaw = [-90 -45 0 45 90];
offset = [-.9 -.6 -.3 0 .3 .6 .9];
success = attitude.status=="success";
fig = newFigure(980,700);
tiles = tiledlayout(fig,2,2,'TileSpacing','compact','Padding','compact');
metrics = {'goal_announcement_s','rmse_distance_m'};
figureTitles = ["Thời gian báo đích (s)","RMSE theo quãng đường (m)"];
for row = 1:2
    valid = attitude.(metrics{row})(success);
    common = [min(valid) max(valid)];
    for m = 1:2
        map = NaN(7,5);
        selected = attitude(attitude.method==methods(m),:);
        for i = 1:height(selected)
            [~,y] = min(abs(offset-selected.y0_m(i)));
            [~,x] = min(abs(yaw-selected.theta0_deg(i)));
            if selected.status(i)=="success"
                map(y,x) = selected.(metrics{row})(i);
            end
        end
        a = nexttile(tiles);
        imagesc(a,yaw,offset,map,'AlphaData',isfinite(map))
        set(a,'YDir','normal','Color',[.88 .88 .88],'XTick',yaw,'YTick',offset)
        clim(a,common); colorbar(a)
        title(a,titles(m)+" · "+figureTitles(row))
        xlabel(a,'Góc hướng ban đầu (°)'); ylabel(a,'Độ lệch y ban đầu (m)')
        for y = 1:7
            for x = 1:5
                value = map(y,x);
                if isnan(value)
                    text(a,yaw(x),offset(y),"×",'HorizontalAlignment','center')
                else
                    label = compose("%.2f",value);
                    if row == 2
                        label = compose("%.3f",value);
                    end
                    c = 'w';
                    if (value-common(1))/diff(common)>.67
                        c = 'k';
                    end
                    text(a,yaw(x),offset(y),label,'Color',c,'FontWeight','bold', ...
                        'HorizontalAlignment','center','FontSize',10)
                end
            end
        end
    end
end
colormap(fig,parula)
saveFigure(fig,folder,'initial_pose_heatmaps')

fig = newFigure(970,690);
tiles = tiledlayout(fig,2,1,'TileSpacing','compact','Padding','compact');
profiles = ["paper_noise","combined_medium","combined_strong"];
cases = ["Chỉ nhiễu","Tổ hợp vừa","Tổ hợp mạnh"];
routes = ["s_curve","corners"];
routeLabels = ["Tuyến chữ S","Tuyến góc ngoặt"];
for i = 1:2
    a = nexttile(tiles); hold(a,'on')
    for m = 1:2
        groups = robust(robust.scenario==routes(i) & robust.method==methods(m) & ...
            ismember(robust.profile,profiles),:);
        [~,idx] = ismember(groups.profile,profiles);
        x = double(idx) + (2*m-3)*.17;
        boxchart(a,x,groups.rmse_distance_m,'BoxWidth',.25,'BoxFaceColor',colors(m,:), ...
            'MarkerColor',colors(m,:),'DisplayName',titles(m))
    end
    title(a,routeLabels(i)); ylabel(a,'RMSE theo quãng đường (m)')
    xticks(a,1:3); xticklabels(a,cases); xlim(a,[.55 3.45]); grid(a,'on')
    if i == 1
        legend(a,'Location','northwest')
    end
end
saveFigure(fig,folder,'robustness_distributions')

fig = newFigure(980,790);
tiles = tiledlayout(fig,3,2,'TileSpacing','compact','Padding','compact');
for m = 1:2
    t = readtable(fullfile(folder,"incident_"+methods(m)+".csv"));
    dt = t.time_s-t.event_time_s(end);
    a = nexttile(tiles,m); hold(a,'on')
    plot(a,dt,t.path_error_m,'Color',colors(1,:),'LineWidth',1.4,'DisplayName','Sai số bám thật')
    plot(a,dt,hypot(t.x_m-t.measured_x_m,t.y_m-t.measured_y_m), ...
        '--','Color',colors(2,:),'LineWidth',1.25,'DisplayName','Sai số định vị')
    title(a,titles(m)); ylabel(a,'Sai số (m)'); incidentAxis(a)
    if m == 1, legend(a,'Location','northwest'); end

    a = nexttile(tiles,m+2); hold(a,'on')
    plot(a,dt,t.command_left_rad_s,'Color',colors(1,:),'DisplayName','Lệnh trái')
    plot(a,dt,t.left_rad_s,'--','Color',colors(1,:),'LineWidth',1.35,'DisplayName','Thực trái')
    plot(a,dt,t.command_right_rad_s,'Color',colors(2,:),'DisplayName','Lệnh phải')
    plot(a,dt,t.right_rad_s,'--','Color',colors(2,:),'LineWidth',1.35,'DisplayName','Thực phải')
    ylabel(a,'Tốc độ bánh (rad/s)'); incidentAxis(a)
    if m == 1, legend(a,'Location','southeast','NumColumns',2); end

    a = nexttile(tiles,m+4); hold(a,'on')
    plot(a,dt,t.slip_x_m_s,'Color',colors(1,:),'LineWidth',1.35,'DisplayName','Nhiễu dọc')
    plot(a,dt,t.slip_y_m_s,'--','Color',colors(2,:),'LineWidth',1.35,'DisplayName','Nhiễu ngang')
    ylabel(a,'Nhiễu vận tốc (m/s)'); xlabel(a,'Thời gian từ khi kích hoạt (s)')
    incidentAxis(a)
    if m == 1, legend(a,'Location','northeast'); end
end
saveFigure(fig,folder,'robustness_incident')
end

function incidentAxis(a)
for event = [0 1 3]
    xline(a,event,':','Color',[.3 .3 .3],'HandleVisibility','off')
end
xlim(a,[-1 5]); grid(a,'on')
end

function fig = newFigure(w,h)
fig = figure('Visible','off','Color','white','Position',[80 80 w h]);
set(fig,'DefaultAxesFontName','Arial','DefaultAxesFontSize',12, ...
    'DefaultTextFontName','Arial','DefaultTextInterpreter','none', ...
    'DefaultLegendInterpreter','none')
end

function saveFigure(fig,folder,name)
exportgraphics(fig,fullfile(folder,[name '.png']),'Resolution',300)
exportgraphics(fig,fullfile(folder,[name '.pdf']),'ContentType','vector')
close(fig)
end

