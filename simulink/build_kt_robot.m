function model = build_kt_robot()
% Build a self-contained model from the editable MATLAB Function sources.
root = fileparts(mfilename('fullpath'));
model = 'KT_Robot_Pure_Pursuit';
if bdIsLoaded(model)
    close_system(model,0)
end
new_system(model);
set_param(model,'SolverType','Fixed-step','Solver','FixedStepDiscrete', ...
    'FixedStep','Ts','StopTime','tEnd','SaveTime','off','SaveOutput','off', ...
    'ReturnWorkspaceOutputs','on','PaperOrientation','landscape','PaperPositionMode','auto');
cfg = pp_config();
workspace = get_param(model,'ModelWorkspace');
keys = fieldnames(cfg);
for i = 1:numel(keys)
    assignin(workspace,keys{i},cfg.(keys{i}))
end

block(model,'simulink/Sources/Constant','Reference path',[30 40 105 70],'Value','pathData');
block(model,'simulink/Sources/Constant','Controller settings',[30 100 105 130],'Value','settings');
block(model,'simulink/Sources/Constant','Robot parameters',[30 160 105 190],'Value','robot');
block(model,'simulink/Sources/Digital Clock','Simulation clock',[30 220 105 250],'SampleTime','Ts');
block(model,'built-in/Subsystem','Pure Pursuit controller',[280 54 425 265], ...
    'BackgroundColor','lightBlue');
controller = [model '/Pure Pursuit controller'];
block(model,'simulink/Sources/Constant','Step size',[481 213 546 243],'Value','Ts');
chart(model,'Differential drive',[614 64 755 146],root,'pp_pose');
block(model,'simulink/Discrete/Unit Delay','Pose at sample k',[607 347 682 383], ...
    'InitialCondition','initialPose','SampleTime','Ts','Orientation','left');
block(model,'simulink/Discrete/Unit Delay','Progress at sample k',[330 323 410 359], ...
    'InitialCondition','0','SampleTime','Ts','Orientation','left');
block(model,'simulink/Sinks/Stop Simulation','Goal or timeout',[475 279 547 315]);
block(model,'simulink/Signal Routing/Mux','Log channels',[807 183 812 283],'Inputs','5');
block(model,'simulink/Sinks/To Workspace','Sample log',[863 213 948 253], ...
    'VariableName','sample_log','SaveFormat','Array');

inputs = {'Path','Settings','Robot','Time','Pose','Previous progress'};
ys = [45 137 294 246 191 91];
for i = 1:numel(inputs)
    block(controller,'built-in/Inport',inputs{i},[30 ys(i) 58 ys(i)+14],'Port',num2str(i));
end
chart(controller,'Look-ahead target',[165 27 366 145],root,'pp_target');
chart(controller,'Pure Pursuit command',[435 48 651 221],root,'pp_command');
chart(controller,'Wheel speeds and limits',[727 65 910 140],root,'pp_wheels');
block(controller,'simulink/Signal Routing/Mux','Tracking channels',[740 187 745 252],'Inputs','5');
outputs = {'Applied twist','Wheel speeds','Tracking data','Progress','Stop'};
ys = [76 118 215 266 304];
for i = 1:numel(outputs)
    block(controller,'built-in/Outport',outputs{i},[985 ys(i) 1013 ys(i)+14],'Port',num2str(i));
end

connect(controller,{
    'Pose/1','Look-ahead target/1'; 'Previous progress/1','Look-ahead target/2';
    'Path/1','Look-ahead target/3'; 'Settings/1','Look-ahead target/4';
    'Pose/1','Pure Pursuit command/1'; 'Look-ahead target/1','Pure Pursuit command/2';
    'Look-ahead target/4','Pure Pursuit command/3'; 'Look-ahead target/5','Pure Pursuit command/4';
    'Time/1','Pure Pursuit command/5'; 'Settings/1','Pure Pursuit command/6';
    'Pure Pursuit command/1','Wheel speeds and limits/1'; 'Robot/1','Wheel speeds and limits/2';
    'Wheel speeds and limits/1','Applied twist/1'; 'Wheel speeds and limits/2','Wheel speeds/1';
    'Look-ahead target/3','Tracking channels/1'; 'Look-ahead target/4','Tracking channels/2';
    'Look-ahead target/2','Tracking channels/3'; 'Look-ahead target/1','Tracking channels/4';
    'Pure Pursuit command/2','Tracking channels/5'; 'Tracking channels/1','Tracking data/1';
    'Look-ahead target/2','Progress/1'; 'Pure Pursuit command/3','Stop/1'
    })
connect(model,{
    'Reference path/1','Pure Pursuit controller/1'; 'Controller settings/1','Pure Pursuit controller/2';
    'Robot parameters/1','Pure Pursuit controller/3'; 'Simulation clock/1','Pure Pursuit controller/4';
    'Pose at sample k/1','Pure Pursuit controller/5'; 'Progress at sample k/1','Pure Pursuit controller/6';
    'Pure Pursuit controller/4','Progress at sample k/1'; 'Pure Pursuit controller/5','Goal or timeout/1';
    'Pose at sample k/1','Differential drive/1'; 'Pure Pursuit controller/1','Differential drive/2';
    'Step size/1','Differential drive/3'; 'Differential drive/1','Pose at sample k/1';
    'Simulation clock/1','Log channels/1'; 'Pose at sample k/1','Log channels/2';
    'Pure Pursuit controller/1','Log channels/3'; 'Pure Pursuit controller/2','Log channels/4';
    'Pure Pursuit controller/3','Log channels/5'; 'Log channels/1','Sample log/1'
    })
save_system(model,fullfile(root,[model '.slx']));
end

function block(parent,source,name,position,varargin)
add_block(source,[parent '/' name],'Position',position,'FontSize','12',varargin{:})
end

function chart(parent,name,position,root,source)
block(parent,'simulink/User-Defined Functions/MATLAB Function',name,position, ...
    'BackgroundColor','lightBlue')
item = find(sfroot,'-isa','Stateflow.EMChart','Path',[parent '/' name]);
item.Script = fileread(fullfile(root,[source '.m']));
end

function connect(system,links)
for i = 1:size(links,1)
    add_line(system,links{i,1},links{i,2},'autorouting','smart');
end
end


