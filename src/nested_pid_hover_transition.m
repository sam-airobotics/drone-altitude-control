%% ================= Drone PID Height Control Dashboard =================
clc; clear;

%% ================= PHYSICS =================
m = 1.25;
g = 9.81;
dt = 0.03;

maxThrust = 2.5 * m * g;
maxVel = 1.5;

%% ================= VELOCITY PID =================
Kp = 8;
Ki = 1.5;
Kd = 4;

maxIntegral = 10;

%% ================= HEIGHT PID =================
Kp_h = 2.0;
Ki_h = 0.6;
Kd_h = 0.8;

int_h = 0;
err_h_prev = 0;

targetHeight = 0;
prev_hover_mode = false;

%% ================= STATES =================
height = 0;
vel = 0;

eInt = 0;
ePrev = 0;

%% ================= HOVER TRANSITION =================
hover_transition = false;
transition_time = 0;

%% ================= LOAD URDF =================
drone = importrobot("drone.urdf");
drone.DataFormat = "struct";
config = homeConfiguration(drone);

%% ================= UI SETUP =================
ui = uifigure('Name','Drone Dashboard','Color',[0.08 0.09 0.11],...
    'Units','normalized','Position',[0.1 0.1 0.8 0.8]);

mainGrid = uigridlayout(ui,[1 2]);
mainGrid.ColumnWidth = {'2x','1x'};

%% ================= SIM PANEL =================
simPanel = uipanel(mainGrid);
ax = axes('Parent', simPanel);

hold(ax,'on'); grid(ax,'on'); view(ax,3);
axis(ax,[-2 2 -2 2 0 5]); axis(ax,'equal'); axis(ax,'vis3d');

set(ax,'Color',[0.1 0.1 0.12],...
    'XColor',[1 1 1],'YColor',[1 1 1],'ZColor',[1 1 1]);

title(ax,'Drone','Color','w');

%% ===== GROUND =====
[X,Y] = meshgrid(-5:0.5:5, -5:0.5:5);
Z = zeros(size(X));
surf(ax, X, Y, Z,'FaceColor',[0.1 0.5 0.1],'EdgeColor','none');

%% ================= CONTROL PANEL =================
panel = uipanel(mainGrid);
panel.Title = 'Control Panel';
panel.BackgroundColor = [0.12 0.13 0.15];

panelGrid = uigridlayout(panel,[7 1]);
panelGrid.RowHeight = {30,40,'3x',40,10,'2x','2x'};

uilabel(panelGrid,'Text','Throttle','FontSize',16,...
    'FontWeight','bold','HorizontalAlignment','center','FontColor',[1 1 1]);

throttleLabel = uilabel(panelGrid,'Text','0.0','FontSize',18,...
    'HorizontalAlignment','center','FontColor',[0.2 0.8 1]);

sliderPanel = uipanel(panelGrid);
sliderGrid = uigridlayout(sliderPanel,[1 1]);

sld = uislider(sliderGrid,'Orientation','vertical',...
    'Limits',[0 100],'Value',0);

hoverBtn = uibutton(panelGrid,'state');
hoverBtn.Text = '☐ Hover Mode';
hoverBtn.Value = false;
hoverBtn.FontSize = 14;

uipanel(panelGrid,'BackgroundColor',[0.5 0.5 0.5]);

statusGrid = uigridlayout(panelGrid,[5 1]);

targetLabel = uilabel(statusGrid,'FontColor',[1 0.8 0.2]);
heightLabel = uilabel(statusGrid,'FontColor',[0.6 1 0.7]);
velLabel = uilabel(statusGrid,'FontColor',[0.4 0.8 1]);
thrustLabel = uilabel(statusGrid,'FontColor',[1 0.6 0.2]);
errorLabel = uilabel(statusGrid,'FontColor',[1 0.4 0.8]);

sld.ValueChangingFcn = @(src,evt) ...
    set(throttleLabel,'Text',sprintf('%.1f',evt.Value));

%% ================= INITIAL DRAW =================
show(drone, config,'Parent', ax,'PreservePlot', false,'FastUpdate', true);
campos(ax,[3 3 3]); camtarget(ax,[0 0 0]);

%% ================= SIMULATION LOOP =================
while true

    if ~isvalid(sld)
        break
    end

    throttle = sld.Value / 100;
    hover_mode = hoverBtn.Value;

    %% ===== HOVER TOGGLE =====
    if hover_mode && ~prev_hover_mode
        targetHeight = height;

        % Reset controllers
        int_h = 0;
        err_h_prev = 0;
        eInt = 0;
        ePrev = 0;

        hover_transition = true;
        transition_time = 0;
    end
    prev_hover_mode = hover_mode;

    %% ===== TRANSITION TIMER =====
    if hover_mode && hover_transition
        transition_time = transition_time + dt;

        if transition_time > 1.0
            hover_transition = false;
        end
    end

    %% ===== BUTTON TEXT =====
    if hover_mode
        hoverBtn.Text = '☑ Hover Mode';
    else
        hoverBtn.Text = '☐ Hover Mode';
    end

    %% ================= CONTROL =================
    if hover_mode
        err_h = targetHeight - height;

        int_h = int_h + err_h * dt;
        der_h = (err_h - err_h_prev) / dt;

        pid_h = Kp_h*err_h + Ki_h*int_h + Kd_h*der_h;

        % Smooth entry
        targetVel = 0.3 * vel + pid_h;

        err_h_prev = err_h;

    else
        % Manual mode
        targetVel = (throttle - 0.5) * 2 * maxVel;

        int_h = 0;
        err_h_prev = 0;
    end

    %% ================= VELOCITY PID =================
    e = targetVel - vel;

    eInt = eInt + e * dt;
    eInt = min(max(eInt, -maxIntegral), maxIntegral);

    eDer = (e - ePrev) / dt;

    pidOutput = Kp * e + Ki * eInt + Kd * eDer;

    %% ================= THRUST =================
    if hover_mode
        thrust = m*g + pidOutput;

        % Limit sudden spike
        thrust = min(thrust, 1.2*m*g);

    else
        thrust = pidOutput;
    end

    thrust = min(max(thrust,0), maxThrust);

    ePrev = e;

    %% ================= DYNAMICS =================
    acc = (thrust - m*g) / m;

    vel = vel + acc * dt;
    height = height + vel * dt;

    %% ===== GROUND CONTACT =====
    if height <= 0
        height = 0;
        if vel < 0
            vel = 0;
        end
    end

    %% ================= UI =================
    if hover_mode
        targetLabel.Text = sprintf('Target Height: %.2f m (AUTO)', targetHeight);
    else
        targetLabel.Text = 'Manual Mode (Throttle)';
    end

    heightLabel.Text = sprintf('Height: %.2f m', height);
    velLabel.Text = sprintf('Velocity: %.2f m/s', vel);
    thrustLabel.Text = sprintf('Thrust: %.2f N', thrust);
    errorLabel.Text = sprintf('Error: %.2f m', targetHeight - height);

    %% ================= VISUAL =================
    for i = 1:length(config)
        if strcmp(config(i).JointName,'z_joint')
            config(i).JointPosition = height;
        end
    end

    show(drone, config,'Parent', ax,...
        'PreservePlot', false,'FastUpdate', true);

    axis(ax,[-2 2 -2 2 0 5]);
    camtarget(ax,[0 0 height]);

    drawnow limitrate;
    pause(dt);
end
