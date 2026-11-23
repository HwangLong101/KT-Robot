function cfg = kt_robust_config(scenario,method,profile,seed,initialPose)
% Build constant inputs and reproduce independent Gaussian samples by seed.
cfg = pp_config(scenario,method,0.55,0.05,120);
cfg.initialPose = initialPose;
cfg.impact = kt_robust_profile(profile,scenario);
cfg.tEnd = cfg.tEnd + 2;
stream = RandStream('mt19937ar','Seed',seed);
cfg.noise = randn(stream,3,round(cfg.tEnd/cfg.Ts)+1);
end

