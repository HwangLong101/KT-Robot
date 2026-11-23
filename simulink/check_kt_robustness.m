function check_kt_robustness()
% Check the individual models and the full no-disturbance Simulink loop.
dt = .05;
impact = kt_robust_profile("delay","corners");
clear kt_robust_sensor
for k = 0:5
    event = [-1;double(k==4 || k==5);0];
    measured = kt_robust_sensor([k;0;0],event,k*dt,dt,impact,zeros(3,6));
    expected = min(max(k-2,0),1);
    assert(isequal(measured,[expected;0;0]),'Delay or sample hold failed.')
end

clear kt_robust_actuator
cfg = pp_config();
impact = kt_robust_profile("motor","corners");
for k = 1:8
    realized = kt_robust_actuator([10;8],cfg.robot,impact,[-1;0;0],(k-1)*dt,dt);
    expected = [10;8]*(1-exp(-k*dt/.2));
    assert(max(abs(realized-expected))<1e-12,'Wheel step response failed.')
end
clear kt_robust_actuator
impact = kt_robust_profile("radius","corners");
[~,body] = kt_robust_actuator([10;10],cfg.robot,impact,[-1;0;0],0,dt);
assert(max(abs(body-[.5;0;-.02/.30]))<1e-12,'Radius mismatch test failed.')
clear kt_robust_actuator
impact = kt_robust_profile("slip","corners");
[~,body,slip] = kt_robust_actuator([0;0],cfg.robot,impact,[1;0;1],1,dt);
expected = [.03*sin(.045408*pi);.05*cos(-.00459*pi)];
assert(isequal(body,[expected;0]) && isequal(slip,expected),'Slip onset test failed.')
assert(max(abs(kt_robust_motion([0;0;0],[0;1;1],pi/2)-[-1;1;pi/2]))<1e-12, ...
    'Lateral displacement integration failed.')

clear kt_robust_truth
for k = 0:61
    [~,event] = kt_robust_truth([3.5+.01*(k>0);0;0],cfg.pathData,k*dt, ...
        kt_robust_profile("combined_medium","corners"),dt);
    if k==0
        assert(isequal(event,[-1;0;0]),'Disturbance triggered at the threshold.')
    else
        assert(isequal(event,[dt;double(k<=10);double(k<=60)]),'Wrong disturbance window.')
    end
end

columns = {'time_s','x_m','y_m','theta_rad','v_m_s','omega_rad_s', ...
    'left_rad_s','right_rad_s','path_error_m','goal_distance_m','progress_m', ...
    'target_x_m','target_y_m','mode_code'};
for scenario = ["straight","s_curve","corners"]
    for method = ["baseline","curvature"]
        reference = run_kt_robot(scenario,method);
        [tested,metrics] = run_kt_robustness(scenario,method,"ideal",1);
        assert(metrics.status=="success" && height(tested)==height(reference), ...
            'Ideal model changed completion.')
        assert(max(abs(tested{:,columns}-reference{:,:}),[],'all')<=1e-7, ...
            'Ideal model changed the nominal trajectory.')
        assert(isequal(tested{:,2:4},tested{:,5:7}),'Ideal sensing was changed.')
    end
end
a = run_kt_robustness("corners","curvature","combined_strong",1);
b = run_kt_robustness("corners","curvature","combined_strong",1);
assert(isequal(a,b),'Repeated seed changed the full simulation trace.')
c = kt_robust_config("corners","baseline","paper_noise",1,[0;-.6;0]);
d = kt_robust_config("corners","curvature","combined_strong",1,[0;-.6;0]);
assert(isequal(c.noise,d.noise),'Paired runs must share the same Gaussian draws.')
assert(sum(a.outage)==20 && sum(a.slip_active)==60,'Strong case has wrong event duration.')
assert(a.event_time_s(end)>0,'Strong case did not include a disturbance event.')
disp('Disturbance timing, response, six ideal comparisons and seed replay passed.')
end

