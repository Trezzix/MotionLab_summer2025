classdef sg_IK_App_exported < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                       matlab.ui.Figure
        ZeroVelLamp                    matlab.ui.control.Lamp
        RTTargetMachineSystemlogLabel  matlab.ui.control.Label
        SwitchtozeroveltokillallvelocitycommandstorobotLabel  matlab.ui.control.Label
        Image                          matlab.ui.control.Image
        SystemLog                      slrealtime.ui.control.SystemLog
        ConnectButton                  slrealtime.ui.control.ConnectButton
        OperationModesManualModeLabel  matlab.ui.control.Label
        MaualStateSwitchDropDown       matlab.ui.control.DropDown
        MaualStateSwitchDropDownLabel  matlab.ui.control.Label
        OperationModeDropDown          matlab.ui.control.DropDown
        OperationModeDropDownLabel     matlab.ui.control.Label
        StartTrajectoryButton        matlab.ui.control.Button
        Zeroref_switch                 matlab.ui.control.Switch
        StatusBar                      slrealtime.ui.control.StatusBar
        RTTargetMachineControlsPanel   matlab.ui.container.Panel
        RunTimeLabel                   matlab.ui.control.Label
        SetStoptimeLabel               matlab.ui.control.Label
        StopTimeEditField              slrealtime.ui.control.StopTimeEditField
        SimulationTimeEditField        slrealtime.ui.control.SimulationTimeEditField
        StartButton                    matlab.ui.control.Button
        StopButton                     matlab.ui.control.Button
        ManualModeControlsPanel        matlab.ui.container.Panel
        WinchPosEditField              matlab.ui.control.NumericEditField
        WinchPosEditFieldLabel         matlab.ui.control.Label
        StartWinchCmdButton            matlab.ui.control.Button
        MaxtrajectoryvelocityEditField  matlab.ui.control.NumericEditField
        MaxtrajectoryvelocityEditFieldLabel  matlab.ui.control.Label
        ZEditField                     matlab.ui.control.NumericEditField
        ZEditFieldLabel                matlab.ui.control.Label
        YEditField                     matlab.ui.control.NumericEditField
        YEditFieldLabel                matlab.ui.control.Label
        XEditField                     matlab.ui.control.NumericEditField
        XEditFieldLabel                matlab.ui.control.Label
        HeaveCompToggleButton          matlab.ui.control.Button
    end

    properties (Access = public)
        tg                 % Speedgoat target object
        model              % Model handle
        modelWS            % Model workspace (unused)

        controlMode (1, 1) double = 1

        x_coordinate_traj (1,1) double = 0
        y_coordinate_traj (1,1) double = 0
        z_coordinate_traj (1,1) double = 0
        new_command_traj  (1,1) double = 0 %Counter
        Traj_method       (1,1) double = 1
        v_max             (1,1) double = 0
        Operation_Mode    (1,1) double = 2
        Manual_State_Switch (1,1) double = 0
        Winch_PosRef      (1,1) double = 0
        Winch_NewCmd      (1,1) double = 0
        State_Mode_Selector (1,1) double = 0
        Winch_EnableHeaveComp (1,1) logical = false;
    end

    properties (Access = private)
        runTimer           % Timer to watch for running state
        stopTimer          % Timer to watch for stopping cleanup
        runID              % current SDI run ID
    end

    methods (Access = private)

        function timerFunc(app, src, evt)
            if strcmp(status(app.tg), 'running')
                % grab and open the latest run in SDI
                 %runObj = Simulink.sdi.getRun(app.tg.SDIRunId);
                 %Simulink.sdi.openRun(runObj);

            elseif strcmp(status(app.tg), 'loaded')
                % run has finished: stop & delete the timer
                stop(src)
                delete(src)
                app.runTimer = [];
            end
        end

        function stopTimerFunc(app, ~, ~)
            if strcmp(status(app.tg), 'loaded')
                stop(app.stopTimer)
                delete(app.stopTimer)
                app.stopTimer = []
            end
        end

        function writeIKParameters(app)
            % Write trajectory parameters back to model
            app.tg.setparam('', 'x_coordinate_traj', app.x_coordinate_traj);
            app.tg.setparam('', 'y_coordinate_traj', app.y_coordinate_traj);
            app.tg.setparam('', 'z_coordinate_traj', app.z_coordinate_traj);
        end
    end

    methods (Access = private)
        % Executed after app creation
        function startupFcn(app, tg, model, pars)
            app.tg         = tg;
            app.model      = model;
            app.controlMode = 1;
            app.runID      = tg.SDIRunId;
            
            % No instrument setup needed—SDI viewer will handle runtime logging
        end

        function StartButtonPushed(app, ~, ~)
            if strcmp(status(app.tg), 'loaded')

                app.tg.setparam('', 'controlMode', app.controlMode);
                app.tg.setparam('', 'Operation_Mode',      app.Operation_Mode);
                app.tg.setparam('', 'State_Mode_Selector', app.State_Mode_Selector);
                app.tg.setparam('', 'Manual_State_Switch', app.Manual_State_Switch);

                app.tg.start('ReloadOnStop', true);
                %Simulink.sdi.loadView('sdi-inverse-kinematics');
%                 run = Simulink.sdi.getRun(app.tg.SDIRunId);
%                 Simulink.sdi.openRun(run);

%                 app.runTimer = timer( ...
%                     'Period',        0.5, ...
%                     'ExecutionMode', 'fixedSpacing', ...
%                     'TimerFcn',      @(src,evt) app.timerFunc(src,evt) );
%                 start(app.runTimer);
            else
                warning("Start should only be pressed in loaded state")
            end
        end

        function StopButtonPushed(app, ~, ~)
            if strcmp(status(app.tg), 'running')
                % stop the real-time run
                app.tg.stop;
                % start cleanup timer
%                 app.stopTimer = timer( ...
%                     "Period", 0.5, ...
%                     "ExecutionMode", "fixedDelay", ...
%                     "TimerFcn", @(~,~) app.stopTimerFunc() );
%                 start(app.stopTimer)
            else
                warning("Stop should only be pressed in running state")
            end
        end

        function Zeroref_switchValueChanged(app, ~, ~)
            st = status(app.tg);
            if any(strcmp(st, {'loaded','running'}))
                val = app.Zeroref_switch.Value;  
        
                switch val
                    case 'Normal Mode'
                        mode = 1;
                        app.ZeroVelLamp.Enable = true;
                    case 'Zero_vel'
                        mode = 2;
                        app.ZeroVelLamp.Enable = false;
                    otherwise
                        mode = 2;
                        app.ZeroVelLamp.Enable = false;
                end

                app.controlMode = mode;
                app.tg.setparam('', 'controlMode', mode);
            end
        end

        function XEditFieldValueChanged(app, ~, ~)
            app.x_coordinate_traj = app.XEditField.Value;
        end

        function YEditFieldValueChangedChanged(app, ~, ~)
            app.y_coordinate_traj = app.YEditField.Value;
        end

        function ZEditFieldValueChanged(app, ~, ~)
            app.z_coordinate_traj = app.ZEditField.Value;
        end

        function MaxtrajectoryvelocityEditFieldValueChanged(app, ~, ~)
            app.v_max = app.MaxtrajectoryvelocityEditField.Value;
        end

        function WinchPosEditFieldValueChanged(app, ~, ~)
            app.Winch_PosRef = app.WinchPosEditField.Value;
        end

        function StartTrajectoryButtonPushed(app, ~, ~)
            % Send trajectory parameters and bump the command counter
            app.tg.setparam('', 'v_max', app.v_max);
            app.tg.setparam('', 'x_coordinate_traj', app.x_coordinate_traj);
            app.tg.setparam('', 'y_coordinate_traj', app.y_coordinate_traj);
            app.tg.setparam('', 'z_coordinate_traj', app.z_coordinate_traj);
            app.new_command_traj = app.new_command_traj + 1;
            app.tg.setparam('', 'new_command_traj', app.new_command_traj);
        end

        function StartWinchCmdButtonPushed(app, ~, ~)
            app.tg.setparam('', 'Winch_PosRef', app.Winch_PosRef);
            app.Winch_NewCmd = app.Winch_NewCmd + 1;
            app.tg.setparam('', 'Winch_NewCmd', app.Winch_NewCmd);
        end

        function MaualStateSwitchDropDownValueChanged(app, ~, ~)
            val = app.MaualStateSwitchDropDown.Value;
            switch val
                case 'Hold Position',       app.Manual_State_Switch = 0;
                case 'Go home',             app.Manual_State_Switch = 1;
                case 'Move above platform', app.Manual_State_Switch = 2;
                case 'Initiate Hooking',    app.Manual_State_Switch = 3;
                case 'Lower payload onto platform',    app.Manual_State_Switch = 4;    
                otherwise,                  app.Manual_State_Switch = 0;
            end
            app.tg.setparam('', 'Manual_State_Switch', app.Manual_State_Switch);
        end

        function OperationModeDropDownValueChanged(app, ~, ~)
            val = app.OperationModeDropDown.Value;
            switch val
                case 'Manual Mode'
                    app.Operation_Mode = 2;
                case 'Manual State Switch'
                    app.Operation_Mode = 1;
                    app.State_Mode_Selector = 2;
                case 'Autonomous Mode'
                    app.Operation_Mode = 1;
                    app.State_Mode_Selector = 1;
                otherwise
                    app.Operation_Mode = 2;
                    app.State_Mode_Selector = 2;
            end
            app.tg.setparam('', 'Operation_Mode', app.Operation_Mode);
            app.tg.setparam('', 'State_Mode_Selector', app.State_Mode_Selector);
        end

        function HeaveCompToggleButtonPushed(app, event)
            app.Winch_EnableHeaveComp = ~app.Winch_EnableHeaveComp;

            % Update the button's appearance (Text and Color)
            if app.Winch_EnableHeaveComp
                app.HeaveCompToggleButton.Text = 'Winch Heave Comp: ON';
                app.HeaveCompToggleButton.BackgroundColor = [0.8, 1.0, 0.8]; % Light green
            else
                app.HeaveCompToggleButton.Text = 'Winch Heave Comp: OFF';
                app.HeaveCompToggleButton.BackgroundColor = [0.94, 0.94, 0.94]; % Default grey
            end

            heaveCompValue = double(app.Winch_EnableHeaveComp); 

            if ~isempty(app.tg) 
                 % This directly attempts to set the parameter. 
                 app.tg.setparam('', 'Winch_EnableHeaveComp', heaveCompValue);
            end
            
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Position = [100 100 694 718];
            app.UIFigure.Name = 'MATLAB App';

            % Create ManualModeControlsPanel
            app.ManualModeControlsPanel = uipanel(app.UIFigure);
            app.ManualModeControlsPanel.Title = 'Manual Mode Controls';
            app.ManualModeControlsPanel.Position = [10 361 302 181];

            % Create HeaveCompToggleButton
            app.HeaveCompToggleButton = uibutton(app.ManualModeControlsPanel, 'push');
            app.HeaveCompToggleButton.Position = [60 19 164 30];
            % Set initial text based on default state (HeaveCompEnabled = false)
            app.HeaveCompToggleButton.Text = 'Winch Heave Comp: OFF'; 
            app.HeaveCompToggleButton.BackgroundColor = [0.94, 0.94, 0.94]; 
            app.HeaveCompToggleButton.ButtonPushedFcn = createCallbackFcn(app, @HeaveCompToggleButtonPushed, true);

            % Create XEditFieldLabel
            app.XEditFieldLabel = uilabel(app.ManualModeControlsPanel);
            app.XEditFieldLabel.HorizontalAlignment = 'right';
            app.XEditFieldLabel.Position = [9 128 15 22];
            app.XEditFieldLabel.Text = 'X:';

            % Create XEditField
            app.XEditField = uieditfield(app.ManualModeControlsPanel, 'numeric');
            app.XEditField.Position = [28 128 39 22];
            app.XEditField.ValueChangedFcn = createCallbackFcn(app, @XEditFieldValueChanged, true);

            % Create YEditFieldLabel
            app.YEditFieldLabel = uilabel(app.ManualModeControlsPanel);
            app.YEditFieldLabel.HorizontalAlignment = 'right';
            app.YEditFieldLabel.Position = [70 128 16 22];
            app.YEditFieldLabel.Text = 'Y:';

            % Create YEditField
            app.YEditField = uieditfield(app.ManualModeControlsPanel, 'numeric');
            app.YEditField.Position = [90 128 39 22];
            app.YEditField.ValueChangedFcn = createCallbackFcn(app, @YEditFieldValueChangedChanged, true);

            % Create ZEditFieldLabel
            app.ZEditFieldLabel = uilabel(app.ManualModeControlsPanel);
            app.ZEditFieldLabel.HorizontalAlignment = 'right';
            app.ZEditFieldLabel.Position = [132 128 15 22];
            app.ZEditFieldLabel.Text = 'Z:';

            % Create ZEditField
            app.ZEditField = uieditfield(app.ManualModeControlsPanel, 'numeric');
            app.ZEditField.Position = [151 128 39 22];
            app.ZEditField.ValueChangedFcn = createCallbackFcn(app, @ZEditFieldValueChanged, true);

            % Create MaxtrajectoryvelocityEditFieldLabel
            app.MaxtrajectoryvelocityEditFieldLabel = uilabel(app.ManualModeControlsPanel);
            app.MaxtrajectoryvelocityEditFieldLabel.HorizontalAlignment = 'right';
            app.MaxtrajectoryvelocityEditFieldLabel.Position = [7 97 147 22];
            app.MaxtrajectoryvelocityEditFieldLabel.Text = 'Max trajectory velocity: m/s';

            % Create MaxtrajectoryvelocityEditField
            app.MaxtrajectoryvelocityEditField = uieditfield(app.ManualModeControlsPanel, 'numeric');
            app.MaxtrajectoryvelocityEditField.Position = [158 97 97 22];
            app.MaxtrajectoryvelocityEditField.ValueChangedFcn = createCallbackFcn(app, @MaxtrajectoryvelocityEditFieldValueChanged, true);

            % Create StartWinchCmdButton
            app.StartWinchCmdButton = uibutton(app.ManualModeControlsPanel, 'push');
            app.StartWinchCmdButton.Position = [197 61 99 22];
            app.StartWinchCmdButton.Text = 'Start Winch Cmd';
            app.StartWinchCmdButton.ButtonPushedFcn = createCallbackFcn(app, @StartWinchCmdButtonPushed, true);

            % Create WinchPosEditFieldLabel
            app.WinchPosEditFieldLabel = uilabel(app.ManualModeControlsPanel);
            app.WinchPosEditFieldLabel.HorizontalAlignment = 'right';
            app.WinchPosEditFieldLabel.Position = [9 61 66 22];
            app.WinchPosEditFieldLabel.Text = 'Winch Pos:';

            % Create WinchPosEditField
            app.WinchPosEditField = uieditfield(app.ManualModeControlsPanel, 'numeric');
            app.WinchPosEditField.Position = [90 61 46 22];
            app.WinchPosEditField.ValueChangedFcn = createCallbackFcn(app, @WinchPosEditFieldValueChanged, true);

            % Create RTTargetMachineControlsPanel
            app.RTTargetMachineControlsPanel = uipanel(app.UIFigure);
            app.RTTargetMachineControlsPanel.Title = 'RT Target Machine Controls';
            app.RTTargetMachineControlsPanel.Position = [10 190 302 158];

            % Create StopButton
            app.StopButton = uibutton(app.RTTargetMachineControlsPanel, 'push');
            app.StopButton.Icon = 'slrtStopIcon.png';
            app.StopButton.Position = [159 11 118 50];
            app.StopButton.Text = 'Stop';
            app.StopButton.ButtonPushedFcn = createCallbackFcn(app, @StopButtonPushed, true);

            % Create StartButton
            app.StartButton = uibutton(app.RTTargetMachineControlsPanel, 'push');
            app.StartButton.Icon = 'slrtRunIcon.png';
            app.StartButton.Position = [18 11 118 50];
            app.StartButton.Text = 'Start';
            app.StartButton.ButtonPushedFcn = createCallbackFcn(app, @StartButtonPushed, true);

            % Create SimulationTimeEditField
            app.SimulationTimeEditField = slrealtime.ui.control.SimulationTimeEditField(app.RTTargetMachineControlsPanel);
            app.SimulationTimeEditField.Position = [116 77 90 20];

            % Create StopTimeEditField
            app.StopTimeEditField = slrealtime.ui.control.StopTimeEditField(app.RTTargetMachineControlsPanel);
            app.StopTimeEditField.Position = [116 105 90 21];

            % Create SetStoptimeLabel
            app.SetStoptimeLabel = uilabel(app.RTTargetMachineControlsPanel);
            app.SetStoptimeLabel.Position = [36 104 81 22];
            app.SetStoptimeLabel.Text = 'Set Stop time:';

            % Create RunTimeLabel
            app.RunTimeLabel = uilabel(app.RTTargetMachineControlsPanel);
            app.RunTimeLabel.Position = [54 76 63 22];
            app.RunTimeLabel.Text = 'Run Time :';

            % Create StatusBar
            app.StatusBar = slrealtime.ui.control.StatusBar(app.UIFigure);
            app.StatusBar.Position = [1 3 613 20];

            % Create Zeroref_switch
            app.Zeroref_switch = uiswitch(app.UIFigure, 'slider');
            app.Zeroref_switch.Items = {'Zero vel', 'Normal Mode'};
            app.Zeroref_switch.Position = [403 155 45 20];
            app.Zeroref_switch.Value = 'Normal Mode';
            app.Zeroref_switch.ValueChangedFcn = createCallbackFcn(app, @Zeroref_switchValueChanged, true);

            % Create StartTrajectoryButton
            app.StartTrajectoryButton = uibutton(app.UIFigure, 'push');
            app.StartTrajectoryButton.Position = [207 489 100 22];
            app.StartTrajectoryButton.Text = 'Start Trajectory';
            app.StartTrajectoryButton.ButtonPushedFcn = createCallbackFcn(app, @StartTrajectoryButtonPushed, true);

            % Create OperationModeDropDownLabel
            app.OperationModeDropDownLabel = uilabel(app.UIFigure);
            app.OperationModeDropDownLabel.HorizontalAlignment = 'right';
            app.OperationModeDropDownLabel.Position = [33 112 91 22];
            app.OperationModeDropDownLabel.Text = 'Operation Mode';

            % Create OperationModeDropDown
            app.OperationModeDropDown = uidropdown(app.UIFigure);
            app.OperationModeDropDown.Items = {'Manual Mode', 'Manual State Switch', 'Autonomous Mode'};
            app.OperationModeDropDown.Position = [139 112 148 22];
            app.OperationModeDropDown.Value = 'Manual Mode';
            app.OperationModeDropDown.ValueChangedFcn = createCallbackFcn(app, @OperationModeDropDownValueChanged, true);

            % Create MaualStateSwitchDropDownLabel
            app.MaualStateSwitchDropDownLabel = uilabel(app.UIFigure);
            app.MaualStateSwitchDropDownLabel.HorizontalAlignment = 'right';
            app.MaualStateSwitchDropDownLabel.Position = [28 153 108 22];
            app.MaualStateSwitchDropDownLabel.Text = 'Maual State Switch';

            % Create MaualStateSwitchDropDown
            app.MaualStateSwitchDropDown = uidropdown(app.UIFigure);
            app.MaualStateSwitchDropDown.Items = {'Hold Position', 'Go home', 'Move above platform', 'Initiate Hooking', 'Lower payload onto platform'};
            app.MaualStateSwitchDropDown.Position = [139 153 148 22];
            app.MaualStateSwitchDropDown.Value = 'Hold Position';
            app.MaualStateSwitchDropDown.ValueChangedFcn = createCallbackFcn(app, @MaualStateSwitchDropDownValueChanged, true);

            % Create OperationModesManualModeLabel
            app.OperationModesManualModeLabel = uilabel(app.UIFigure);
            app.OperationModesManualModeLabel.Position = [17 35 647 54];
            app.OperationModesManualModeLabel.Text = {'Operation Modes'; 'Manual Mode: Control the robot end effector by input xyz coordinates'; 'Manual State Switch: Use the dropdown menu to control the robot between the states in the statemachine'; 'Autonomous Mode: Robot runs through the state machine hooking and unhooking the payload without user interatcion'};

            % Create ConnectButton
            app.ConnectButton = slrealtime.ui.control.ConnectButton(app.UIFigure);
            app.ConnectButton.Position = [94 3 100 20];

            % Create SystemLog
            app.SystemLog = slrealtime.ui.control.SystemLog(app.UIFigure);
            app.SystemLog.Position = [335 190 327 488];

            % Create Image
            app.Image = uiimage(app.UIFigure);
            app.Image.Position = [10 577 296 122];
            app.Image.ImageSource = 'motion-lab.png';

            % Create SwitchtozeroveltokillallvelocitycommandstorobotLabel
            app.SwitchtozeroveltokillallvelocitycommandstorobotLabel = uilabel(app.UIFigure);
            app.SwitchtozeroveltokillallvelocitycommandstorobotLabel.Position = [361 110 166 27];
            app.SwitchtozeroveltokillallvelocitycommandstorobotLabel.Text = {'Switch to zero vel to kill'; 'all velocity commands to robot'};

            % Create RTTargetMachineSystemlogLabel
            app.RTTargetMachineSystemlogLabel = uilabel(app.UIFigure);
            app.RTTargetMachineSystemlogLabel.Position = [335 677 166 22];
            app.RTTargetMachineSystemlogLabel.Text = 'RT Target Machine Systemlog';

            % Create ZeroVelLamp
            app.ZeroVelLamp = uilamp(app.UIFigure);
            app.ZeroVelLamp.Position = [551 112 63 63];

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end

    methods (Access = public)
        function app = sg_IK_App_exported(varargin)
            createComponents(app);
            registerApp(app, app.UIFigure);
            runStartupFcn(app, @(app)startupFcn(app, varargin{:}));
            if nargout == 0
                clear app;
            end
        end

        function delete(app)
            % Delete UIFigure when app is deleted
            delete(app.UIFigure);
        end
    end
end
