clear; close all; clc;
Ts = 1e-3;

model = 'sg_IK';
open_system(model);
%% Initial values
modelWorkspaceParameters;
modelWS = get_param(model, 'ModelWorkspace');
modelWSList = whos(modelWS);
for i = 1:numel(modelWSList)
    name = modelWSList(i).name;
    assignin(modelWS, name, pars.(name));
end



%% Save
save_system(model);
%% Build
evalc('slbuild(model)');
bdclose(model);