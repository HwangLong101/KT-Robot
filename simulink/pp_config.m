function cfg = pp_config(scenario,method,lookahead,dt,maxTime)
% Build model inputs in SI units. scenario can also be an N-by-2 polyline.
arguments
    scenario = "corners"
    method (1,1) string {mustBeMember(method,["baseline","curvature"])} = "baseline"
    lookahead (1,1) double {mustBePositive,mustBeFinite} = 0.55
    dt (1,1) double {mustBePositive,mustBeFinite} = 0.05
    maxTime (1,1) double {mustBePositive,mustBeFinite} = 120
end
assert(dt <= maxTime,'Sample time must not exceed maximum duration.')
if isnumeric(scenario)
    points = double(scenario);
else
    switch string(scenario)
        case "straight"
            points = [0 0; 8 0];
        case "s_curve"
            x = linspace(0,10,201).';
            points = [x sin(2*pi*x/10)];
        case "corners"
            points = [0 0; 4 0; 4 3; 8 3; 8 6];
        otherwise
            error('Unknown scenario: %s',string(scenario))
    end
end
validateattributes(points,{'double'},{'2d','ncols',2,'finite','real'})
assert(size(points,1)>=2,'A path needs at least two waypoints.')
d = diff(points);
segments = hypot(d(:,1),d(:,2));
assert(all(segments>=1e-9),'Consecutive duplicate waypoints are unsupported.')
cfg.pathData = [points [0;cumsum(segments)] [segments;0]];
cfg.initialPose = [0;-0.6;0];
cfg.robot = [0.05;0.30;12;2];
cfg.Ts = dt;
% Lookahead, speed, goal tolerance/gain, alignment threshold/gain,
% policy, lateral demand limit, sample time and last sample index.
cfg.settings = [lookahead;0.50;0.05;1;deg2rad(75);2;double(method=="curvature"); ...
    0.12;dt;floor(maxTime/dt)];
cfg.tEnd = cfg.settings(10)*dt;
end

