function model = build_kt_robustness()
% Build the disturbance model and copy the verified Pure Pursuit subsystem.
root = fileparts(mfilename('fullpath'));
model = 'KT_Robot_Robustness';
nominal = 'KT_Robot_Pure_Pursuit';
if bdIsLoaded(model)
    close_system(model,0)
end
if ~bdIsLoaded(nominal)
    load_system(fullfile(root,[nominal '.slx']))
end
new_system(model)
set_param(model,'SolverType','Fixed-step','Solver','FixedStepDiscrete', ...
    'FixedStep','Ts','StopTime','tEnd','SaveTime','off','SaveOutput','off', ...
    'ReturnWorkspaceOutputs','on','PaperOrientation','landscape','PaperPositionMode','auto');
cfg = kt_robust_config("corners","baseline","combined_strong",1,[0;-.6;0]);
workspace = get_param(model,'ModelWorkspace');
keys = fieldnames(cfg);
for i = 1:numel(keys)
    assignin(workspace,keys{i},cfg.(keys{i}))
end

constant(model,'Reference path','pathData',[25 40 95 64],'path')
constant(model,'Controller parameters','settings',[25 86 95 110],'settings')
constant(model,'Robot parameters','robot',[25 132 95 156],'robot')
constant(model,'Disturbance parameters','impact',[25 178 95 202],'impact')
constant(model,'Gaussian samples','noise',[25 224 95 248],'noise')
constant(model,'Step size','Ts',[25 270 95 294],'dt')
add(model,'simulink/Sources/Digital Clock','Simulation clock',[25 316 95 340],'SampleTime','Ts')
send(model,'Simulation clock/1','time',124,328)

chart(model,'Localization',[326 68 508 241],root,'kt_robust_sensor','lightBlue');
add(model,[nominal '/Pure Pursuit controller'],'Pure Pursuit controller',[625 62 800 242]);
set_param([model '/Pure Pursuit controller/Tracking channels'],'Inputs','[1 1 1 2 1]');
chart(model,'Goal decision latch',[917 100 1080 224],root,'kt_robust_command','lightBlue');
chart(model,'Motor and slip model',[1197 57 1402 239],root,'kt_robust_actuator','lightBlue');
chart(model,'Integrate true pose',[1520 63 1693 175],root,'kt_robust_motion','lightBlue');
chart(model,'True path geometry',[326 392 533 542],root,'kt_robust_truth','lightBlue');
chart(model,'Settling check',[918 392 1123 542],root,'kt_robust_stop','lightBlue');
add(model,'simulink/Discrete/Unit Delay','Controller progress',[664 306 745 335], ...
    'InitialCondition','0','SampleTime','Ts','Orientation','left');
add(model,'simulink/Discrete/Unit Delay','True pose state',[1580 250 1655 285], ...
    'InitialCondition','initialPose','SampleTime','Ts');
add(model,'simulink/Sinks/Stop Simulation','Stop after settling',[1161 449 1237 485]);
add(model,'simulink/Sinks/Terminator','Nominal body demand',[819 69 839 89],'ShowName','off');
add(model,'simulink/Signal Routing/Mux','Log channels',[1487 374 1492 616],'Inputs','[1 3 3 2 2 3 3 3 2 2 6]');
add(model,'simulink/Sinks/To Workspace','Robustness log',[1548 472 1656 507], ...
    'VariableName','robust_log','SaveFormat','Array');

join(model,'Localization/1','Pure Pursuit controller/5')
join(model,'Pure Pursuit controller/2','Goal decision latch/1')
join(model,'Pure Pursuit controller/3','Goal decision latch/2')
join(model,'Pure Pursuit controller/1','Nominal body demand/1')
add(model,'simulink/Sinks/Terminator','Controller stop request',[820 255 840 275],'ShowName','off');
join(model,'Pure Pursuit controller/5','Controller stop request/1')
join(model,'Pure Pursuit controller/4','Controller progress/1')
join(model,'Controller progress/1','Pure Pursuit controller/6')
join(model,'Goal decision latch/1','Motor and slip model/1')
join(model,'Motor and slip model/2','Integrate true pose/2')
join(model,'Integrate true pose/1','True pose state/1')
join(model,'Settling check/1','Stop after settling/1')

send(model,'True pose state/1','truePose',1685,267)
send(model,'True path geometry/1','geometry',555,430)
send(model,'True path geometry/2','event',555,504)
send(model,'Localization/1','measPose',533,249)
send(model,'Goal decision latch/1','cmdWheel',1090,40)
send(model,'Goal decision latch/2','terminal',1090,254)
send(model,'Motor and slip model/1','wheelActual',1415,19)
send(model,'Motor and slip model/2','bodyActual',1415,251)
send(model,'Motor and slip model/3','slip',1415,215)
send(model,'Pure Pursuit controller/3','ctrlData',820,225)

receive(model,'Localization',{'truePose','event','time','dt','impact','noise'},1:6);
receive(model,'Pure Pursuit controller',{'path','settings','robot','time'},1:4);
receive(model,'Goal decision latch',{'time'},3);
receive(model,'Motor and slip model',{'robot','impact','event','time','dt'},2:6);
receive(model,'Integrate true pose',{'truePose','dt'},[1 3]);
receive(model,'True path geometry',{'truePose','path','time','impact','dt'},1:5);
receive(model,'Settling check',{'terminal','wheelActual','event','time','dt'},1:5);
receive(model,'Log channels',{'time','truePose','measPose','cmdWheel','wheelActual', ...
    'bodyActual','geometry','event','terminal','slip','ctrlData'},1:11);
join(model,'Log channels/1','Robustness log/1')
save_system(model,fullfile(root,[model '.slx']));
end

function add(sys,source,name,pos,varargin)
add_block(source,[sys '/' name],'Position',pos,'FontSize','12',varargin{:})
end

function chart(sys,name,pos,root,source,color)
add(sys,'simulink/User-Defined Functions/MATLAB Function',name,pos,'BackgroundColor',color)
item = find(sfroot,'-isa','Stateflow.EMChart','Path',[sys '/' name]);
item.Script = fileread(fullfile(root,[source '.m']));
% Fixed-width ports break dimension inheritance around the signal feedback.
switch source
    case 'kt_robust_sensor'
        dimensions = {'pose',3;'event',3;'measured',3};
    case 'kt_robust_truth'
        dimensions = {'pose',3;'geometry',3;'event',3};
    case 'kt_robust_command'
        dimensions = {'wheels',2;'tracking',6;'command',2;'terminal',2};
    case 'kt_robust_actuator'
        dimensions = {'command',2;'robot',4;'impact',10;'event',3; ...
            'realWheels',2;'body',3;'slip',2};
    case 'kt_robust_motion'
        dimensions = {'pose',3;'body',3;'nextPose',3};
    case 'kt_robust_stop'
        dimensions = {'terminal',2;'wheels',2;'event',3};
end
for i = 1:size(dimensions,1)
    data = find(item,'-isa','Stateflow.Data','Name',dimensions{i,1});
    data.Props.Array.Size = num2str(dimensions{i,2});
end
end

function constant(sys,name,value,pos,tag)
add(sys,'simulink/Sources/Constant',name,pos,'Value',value)
send(sys,[name '/1'],tag,pos(3)+29,mean(pos([2 4])))
end

function join(sys,source,target)
add_line(sys,source,target,'autorouting','smart');
end

function send(sys,source,tag,x,y)
name = ['Send ' tag];
add(sys,'simulink/Signal Routing/Goto',name,[x y-8 x+75 y+8], ...
    'GotoTag',tag,'ShowName','off')
join(sys,source,[name '/1'])
end

function receive(sys,target,tags,ports)
handles = get_param([sys '/' target],'PortHandles');
for i = 1:numel(ports)
    pos = get_param(handles.Inport(ports(i)),'Position');
    x = pos(1);
    y = pos(2);
    name = [target ' input ' num2str(ports(i))];
    add(sys,'simulink/Signal Routing/From',name,[x-92 y-9 x-24 y+9], ...
        'GotoTag',tags{i},'IconDisplay','Tag','ShowName','off','FontSize','9')
    join(sys,[name '/1'],[target '/' num2str(ports(i))])
end
end





