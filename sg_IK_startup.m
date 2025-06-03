clear; close all; clc;
%Add custom libraries
addpath("mr\")
addpath("Helper_functions\")

Ts = 1e-3;   % Sample-period
Tf = inf;     % Stop-time (for stimulation)
model = 'sg_IK';
open_system(model);
tg = slrealtime('Speedgoat');
stop(tg);
tg.load(model);
Simulink.sdi.view;
Simulink.sdi.loadView('sdi-inverse-kinematics');

%% Update parameters
load pars.mat;
modelWS = get_param(model, 'ModelWorkspace');
modelWSList = whos(modelWS);
for i = 1:numel(modelWSList)
    name = modelWSList(i).name;
    type = modelWSList(i).class;
    if (strcmp(type, 'double'))
        tg.setparam('', name, pars.(name));
    elseif (strcmp(type, 'single'))
        tg.setparam('', name, single(pars.(name)))
    else
        warning("Could not set {name}");
    end
end
%% Call app
app = sg_IK_App_exported(tg, model, pars)