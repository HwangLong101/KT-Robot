function export_kt_robustness(folder)
% Export the native Simulink diagram of the extended loop.
root = fileparts(mfilename('fullpath'));
if nargin == 0
    folder = fullfile(root,'outputs_robustness');
end
if ~isfolder(folder)
    mkdir(folder)
end
model = 'KT_Robot_Robustness';
load_system(fullfile(root,[model '.slx']))
open_system(model)
print(['-s' model],'-dpng',fullfile(folder,'simulink_robustness.png'))
print(['-s' model],'-dpdf',fullfile(folder,'simulink_robustness.pdf'))
end

