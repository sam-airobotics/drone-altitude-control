%% ================= DRONE DASHBOARD (FINAL FIXED) =================
clc; clear;

%% ================= PHYSICS =================
m = 1.25;
g = 9.81;
dt = 0.03;
maxVel = 1.5;

%% ================= MOTOR =================
RPM = 0;
tau = 0.15;
RPM_max = 9000;

k_thrust = 1e-6;
thrust_max = k_thrust * RPM_max^2;
thrust_min = 0;

%% ================= PID =================
Kp = 8; Ki = 1.5; Kd = 4;
Kp_h = 2; Ki_h = 0.6; Kd_h = 0.8;
maxIntegral = 10;

%% ================= STATES =================
height = 0; vel = 0;
eInt = 0; ePrev = 0;
int_h = 0; err_h_prev = 0;
targetHeight = 0;

lastHoverState = false;

%% ================= LOAD URDF =================
useURDF = true;
try
    drone = importrobot("drone.urdf");
    drone.DataFormat = "struct";
    config = homeConfiguration(drone);
catch
    useURDF = false;
end

%% ================= UI =================
ui = uifigure('Name','Drone Dashboard',...
    'Color',[0.08 0.09 0.11],'Position',[100 100 1200 650]);

mainGrid = uigridlayout(ui,[1 2]);
mainGrid.ColumnWidth = {'3x','1.2x'};

%% ================= 3D VIEW =================
simPanel = uipanel(mainGrid);
ax = axes(simPanel);

hold(ax,'on'); grid(ax,'on'); view(ax,3);

% LOCK AXIS
xlim(ax,[-2 2]);
ylim(ax,[-2 2]);
zlim(ax,[0 5]);
axis(ax,'manual');
axis(ax,'equal');

set(ax,'Color',[0.1 0.1 0.12],...
    'XColor','w','YColor','w','ZColor','w');

title(ax,'Drone Simulation','Color','w');

%% ===== GROUND =====
[X,Y] = meshgrid(-2:0.2:2, -2:0.2:2);
Z = zeros(size(X));
ground = surf(ax,X,Y,Z,...
    'FaceColor',[0.1 0.5 0.1],...
    'EdgeColor','none');

%% ===== CAMERA =====
campos(ax,[3 3 3]);
camtarget(ax,[0 0 1]);

%% ===== FALLBACK DRONE =====
if ~useURDF
    droneBody = plot3(ax,0,0,0,'o',...
        'MarkerSize',10,...
        'MarkerFaceColor','cyan');
end

%% ================= CONTROL PANEL =================
panel = uipanel(mainGrid,'Title','Control Panel');
panel.BackgroundColor = [0.12 0.13 0.15];

gridPanel = uigridlayout(panel,[10 1]);
gridPanel.RowHeight = {30,40,120,50,10,30,30,30,30,'1x'};

uilabel(gridPanel,'Text','THROTTLE','FontSize',16,...
    'FontWeight','bold','HorizontalAlignment','center','FontColor','w');

throttleLabel = uilabel(gridPanel,'Text','0%',...
    'FontSize',22,'FontWeight','bold',...
    'HorizontalAlignment','center','FontColor',[0.2 0.8 1]);

sld = uislider(gridPanel,'Orientation','vertical',...
    'Limits',[0 100],'Value',0);

hoverBtn = uibutton(gridPanel,'state','Text','☐ Hover Mode');

uipanel(gridPanel,'BackgroundColor',[0.4 0.4 0.4]);

targetLabel = uilabel(gridPanel,'Text','Target: -','FontColor','w');
heightLabel = uilabel(gridPanel,'Text','Height: 0','FontColor',[0.6 1 0.7]);
velLabel = uilabel(gridPanel,'Text','Velocity: 0','FontColor',[0.4 0.8 1]);
thrustLabel = uilabel(gridPanel,'Text','Thrust: 0','FontColor',[1 0.6 0.2]);
rpmLabel = uilabel(gridPanel,'Text','RPM: 0','FontColor',[0.9 0.9 0.9]);

sld.ValueChangingFcn = @(src,evt) ...
    set(throttleLabel,'Text',sprintf('%.0f%%',evt.Value));

%% ================= LOOP =================
lastTime = tic;

while isvalid(ui)

    if toc(lastTime) < dt
        drawnow;
        continue;
    end
    lastTime = tic;

    throttle = sld.Value / 100;
    hover_mode = hoverBtn.Value;

    %% ===== HOVER SWITCH =====
    if hover_mode ~= lastHoverState
        if hover_mode
            targetHeight = height;
            int_h = 0; err_h_prev = 0;
            eInt = 0; ePrev = 0;
        end
        lastHoverState = hover_mode;
    end

    % FIXED (NO STRING ERROR)
    if hover_mode
        hoverBtn.Text = '☑ Hover Mode';
    else
        hoverBtn.Text = '☐ Hover Mode';
    end

    %% ===== HEIGHT PID =====
    if hover_mode
        err_h = targetHeight - height;
        int_h = int_h + err_h*dt;
        der_h = (err_h - err_h_prev)/dt;

        pid_h = Kp_h*err_h + Ki_h*int_h + Kd_h*der_h;
        targetVel = pid_h;

        err_h_prev = err_h;
    else
        targetVel = (throttle - 0.5)*2*maxVel;
    end

    %% ===== VELOCITY PID =====
    e = targetVel - vel;

    eInt = eInt + e*dt;
    eInt = min(max(eInt,-maxIntegral),maxIntegral);

    eDer = (e - ePrev)/dt;
    pidOutput = Kp*e + Ki*eInt + Kd*eDer;
    ePrev = e;

    %% ===== MOTOR =====
    RPM_target = throttle * RPM_max;
    RPM = RPM + (RPM_target - RPM)*dt/tau;

    thrust = k_thrust * RPM^2 + pidOutput;
    thrust = min(max(thrust,thrust_min),thrust_max);

    %% ===== DYNAMICS =====
    acc = (thrust - m*g)/m;

    vel = vel + acc*dt;
    height = height + vel*dt;

    if height <= 0
        height = 0;
        vel = max(vel,0);
    end

    %% ===== UI =====
    targetLabel.Text = sprintf('Target: %.2f m',targetHeight);
    heightLabel.Text = sprintf('Height: %.2f m',height);
    velLabel.Text = sprintf('Velocity: %.2f m/s',vel);
    thrustLabel.Text = sprintf('Thrust: %.2f N',thrust);
    rpmLabel.Text = sprintf('RPM: %.0f',RPM);

    %% ===== VISUAL =====
    if useURDF
        for i = 1:length(config)
            if strcmp(config(i).JointName,'z_joint')
                config(i).JointPosition = height * 3;
            end
        end

        show(drone, config,...
            'Parent', ax,...
            'PreservePlot', true,...
            'FastUpdate', true);
    else
        set(droneBody,'XData',0,'YData',0,'ZData',height);
    end

    % LOCK AXIS AGAIN
    xlim(ax,[-2 2]);
    ylim(ax,[-2 2]);
    zlim(ax,[0 5]);

    uistack(ground,'bottom');

    drawnow;
end
