function export_kt_diagrams(outputFolder)
% Export the actual root and controller diagrams using Simulink printing.
root = fileparts(mfilename('fullpath'));
if nargin == 0
    outputFolder = fullfile(root,'outputs');
end
if ~isfolder(outputFolder)
    mkdir(outputFolder)
end
model = 'KT_Robot_Pure_Pursuit';
load_system(fullfile(root,[model '.slx']));
systems = {model,[model '/Pure Pursuit controller']};
names = {'simulink_closed_loop','simulink_controller'};
open_system(model);
for i = 1:numel(systems)
    print(['-s' systems{i}],'-dpng',fullfile(outputFolder,[names{i} '.png']));
    print(['-s' systems{i}],'-dpdf',fullfile(outputFolder,[names{i} '.pdf']));
end
end

