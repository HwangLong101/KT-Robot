function impact = kt_robust_profile(profile, scenario)
% [noise x, noise y, noise heading, delay samples, outage seconds,
% motor time constant, left radius bias, right radius bias, slip scale, trigger m].
arguments
    profile (1,1) string {mustBeMember(profile,["ideal","delay","outage", ...
        "motor","radius","slip","paper_noise","combined_medium","combined_strong"])}
    scenario (1,1) string {mustBeMember(scenario,["straight","s_curve","corners"])}
end
impact = zeros(10,1);
impact(10) = 2;
if scenario == "corners"
    impact(10) = 3.5;
end
switch profile
    case "delay"
        impact(4) = 2;
    case "outage"
        impact(5) = 0.5;
    case "motor"
        impact(6) = 0.2;
    case "radius"
        impact(7:8) = [.02;-.02];
    case "slip"
        impact(9) = 1;
    case "paper_noise"
        impact(1:3) = [0;.10;deg2rad(10)];
    case "combined_medium"
        impact(1:9) = [0;.03;deg2rad(3);2;.5;.2;.02;-.02;1];
    case "combined_strong"
        impact(1:9) = [0;.10;deg2rad(10);4;1;.4;.04;-.04;2];
end
end

