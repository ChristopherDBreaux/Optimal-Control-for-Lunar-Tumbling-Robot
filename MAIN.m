% This script handles the optimization, simulation, and plotting for three
% of JOOEE's single-axis maneuvers
% 1. Swing-Up: Wind-up, brake, and balance at the unstable equilibrium
% 2. Slow-Fall: Fall from the unstable eq and gently settle on the ground
% 3. Slow-Step Wind-up, brake, fall and gently settle on the ground

%% OPTIMIZER
% Run this section to solve a batch of optimizations for the inputs below

delete(gcp('nocreate'))
close all
clear all
clc
% ADD GUROBI AND YALMIP PATH HERE
addpath(genpath('C:\gurobi911'))
addpath(genpath('C:\YALMIP-master'))
yalmip ('clear')

% INPUTS
% Each element of the input vector is a structured array of optimization
% parameters. Each input in the list will run in its own thread up to the
% number of CPU cores. I recommend leaving 1 or 2 cores unused so your
% computer is still usable while the code runs

inputs = [];
% Uncomment or add input elements below with the following structure
% mode: JOOEE single-axis maneuver 'swingup', 'slowfall', or 'slowstep'
% objective: cost function 'time' or 'energy'
% N: number of time steps
% s: step size (s)
% xi: initial state [body angle, body ang vel, wheel angle, wheel ang vel]
% xf: final state / impact velocity threshold [body angle, body ang vel]
% maxT: maximum torque constraint (Nm)
% Kw: windup torque multiplier, unused in slowfall

% Quick Test: Swingup, Slowfall, Slowstep
inputs = [inputs, struct('mode','swingup', 'objective','time', 'N',10, 's',0.1, 'xi',[-pi/2; 0; 0; 0], 'xf',[0; 0], 'maxT',1, 'Kw',5)];
inputs = [inputs, struct('mode','slowfall', 'objective','time', 'N',10, 's',0.1, 'xi',[0; 0; 0; 0], 'xf',[pi/2; 3], 'maxT',1, 'Kw',5)];
inputs = [inputs, struct('mode','slowstep', 'objective','time', 'N',15, 's',0.1, 'xi',[-pi/2; 0; 0; 0], 'xf',[pi/2; 3], 'maxT',1, 'Kw',5)];

% Swing Up
% inputs = [inputs, struct('mode','swingup', 'objective','time', 'N',10, 's',0.1, 'xi',[-pi/2; 0; 0; 0], 'xf',[0; 0], 'maxT',1, 'Kw',5)];
% inputs = [inputs, struct('mode','swingup', 'objective','energy', 'N',10, 's',0.1, 'xi',[-pi/2; 0; 0; 0], 'xf',[0; 0], 'maxT',1, 'Kw',5)];
% inputs = [inputs, struct('mode','swingup', 'objective','time', 'N',20, 's',0.05, 'xi',[-pi/2; 0; 0; 0], 'xf',[0; 0], 'maxT',1, 'Kw',5)];
% inputs = [inputs, struct('mode','swingup', 'objective','energy', 'N',20, 's',0.05, 'xi',[-pi/2; 0; 0; 0], 'xf',[0; 0], 'maxT',1, 'Kw',5)];

% Slow Step
% inputs = [inputs, struct('mode','slowstep', 'objective','time', 'N',13, 's',0.1, 'xi',[-pi/2; 0; 0; 0], 'xf',[pi/2; 3], 'maxT',1, 'Kw',5)];
% inputs = [inputs, struct('mode','slowstep', 'objective','energy', 'N',13, 's',0.1, 'xi',[-pi/2; 0; 0; 0], 'xf',[pi/2; 3], 'maxT',1, 'Kw',5)];

% Slow fall
% inputs = [inputs, struct('mode','slowfall', 'objective','time', 'N',40, 's',0.05, 'xi',[0; 0; 0; 0], 'xf',[pi/2; 2], 'maxT',1, 'Kw',5)];
% inputs = [inputs, struct('mode','slowfall', 'objective','energy', 'N',40, 's',0.05, 'xi',[0; 0; 0; 0], 'xf',[pi/2; 2], 'maxT',1, 'Kw',5)];





% Create folder in data with current time stamp to store batch outputs
folder_name = "data/" + strjoin(string(clock),"_");
mkdir(folder_name);

% Create thread for each input element
threads = length(inputs);
parpool(threads);

% Parallel loop
parfor thread = 1:threads
    % Check selected mode and run corresponding optimizer
    if strcmp(inputs(thread).mode,'swingup')
        swingup_opt(inputs(thread), folder_name, thread)
    elseif strcmp(inputs(thread).mode,'slowfall')
        slowfall_opt(inputs(thread), folder_name, thread)
    elseif strcmp(inputs(thread).mode,'slowstep')  
        slowstep_opt(inputs(thread), folder_name, thread)
    end
end

%% PLOTTER
% Run this section to plot a batch of optimizations from the data folder
delete(gcp('nocreate'))
close all
clear all
clc

% Select folder using a file explorer pop-up
disp("Select a folder from the popup window")
folder = uigetdir('data','Select a Batch of Optimizations');
files = dir(fullfile(folder, '*.mat'));
disp("Loading " + string(length(files)) + " files from " + string(folder))

% Get screen properties for window size
screen = get(0,'ScreenSize');
screenx = screen(3);
screeny = screen(4);
figb = screenx*0.3;
figh = screeny*0.9;
figx = (screenx-figb)/2;
figy = (screeny-figh)/4;

% Loop through each file in the folder
threads = length(files);
for thread = 1:threads
    % Extract data from file
    data = load(folder + "/" + string(files(thread).name));    
    formatted_solution = data.formatted_solution;
    mode = formatted_solution.input.mode;
    % Check selected mode and run corresponding plotter
    if strcmp(mode,'swingup')
        % Plot iff there is a valid solution
        if formatted_solution.solved
            % Gather all the data
            [t_sol,x_sol,u_sol,t_w,x_w,u_w,t_b,x_b,u_b] = hybrid_sim(formatted_solution);
            [maxT, mb, Ibcom, mw, Iw, l, Ib, g, a1, a2] = get_properties();
            xi = formatted_solution.input.xi;
            xf = formatted_solution.input.xf;
            % CoAM and CoE based swing-up wheel ang vel
            ww_swing = sqrt(2*l*g*(mb+mw)*(1-cos(xi(1)))*(a1+Iw)/(Iw*Iw));
            maxT = formatted_solution.input.maxT;
            KwMaxT = maxT*formatted_solution.input.Kw;
            
            disp("    " + string(thread) + ". Plotting " + string(files(thread).name))
            % Create Figure
            figure('Position', [figx figy figb figh],'Name',files(thread).name,'NumberTitle','off')
            tiles = tiledlayout(5,1,'TileSpacing','compact','Padding','compact');
            %title(tiles,string(mode)+": "+string(formatted_solution.input.objective),'FontWeight','bold')
            xlabel(tiles,'Time (s)')
            sb = formatted_solution.step_brake;
            % Body Angle
            nexttile; plot([t_sol(1:sb),t_sol(sb:end)],[x_sol(1:sb,1);x_sol(sb+1:end,1)],'Marker','.','color',[0 0.4470 0.7410]); title('Body Angle')
            hold on; plot(t_w,x_w(:,1),t_b,x_b(:,1),'color',[0.8500 0.3250 0.0980])
            hold on; plot([0,t_sol(end)],[0,0],'k:')
            ax = gca; ax.TitleHorizontalAlignment = 'left';
            legend('Solution','Simulation','Location','southeast')
            text(t_sol(end)/20,xi(1),'ground')
            text(t_sol(end)/20,0,'equilibrium')
            ylabel('Angle (rad)')
            % Body Angular Velocity
            nexttile; plot([t_sol(1:sb),t_sol(sb:end)],[x_sol(1:sb,2);x_sol(sb+1:end,2)],'Marker','.','color',[0 0.4470 0.7410]); title('Body Angular Velocity')
            hold on; plot([t_w;t_b],[x_w(:,2);x_b(:,2)],'color',[0.8500 0.3250 0.0980])
            hold on; plot([0,t_sol(end)],[0,0],'k:')
            ax = gca; ax.TitleHorizontalAlignment = 'left';
            ylabel('Angular Velocity (rad/s)')
            % Wheel Angle
            nexttile; plot(t_sol(1:sb),x_sol(1:sb,3),t_sol(sb:end),x_sol(sb+1:end,3),'Marker','.','color',[0 0.4470 0.7410]); title('Wheel Angle')
            hold on; plot(t_w,x_w(:,3),t_b,x_b(:,3),'color',[0.8500 0.3250 0.0980])
            ax = gca; ax.TitleHorizontalAlignment = 'left';
            ylabel('Angle (rad)')
            % Wheel Angular Velocity
            nexttile; plot([t_sol(1:sb),t_sol(sb:end)],[x_sol(1:sb,4);x_sol(sb+1:end,4)],'Marker','.','color',[0 0.4470 0.7410]); title('Wheel Angular Velocity')
            hold on; plot([t_w;t_b],[x_w(:,4);x_b(:,4)],'color',[0.8500 0.3250 0.0980])
            hold on; plot([0,t_sol(end)],[ww_swing,ww_swing],'k:')
            ax = gca; ax.TitleHorizontalAlignment = 'left';
            text(t_sol(end)/20,ww_swing,'\omega_s_w_i_n_g')
            ylabel('Angular Velocity (rad/s)')
            % Input Motor Torque
            nexttile; stairs(t_sol,u_sol,'Marker','.','color',[0 0.4470 0.7410]); title('Motor Torque')
            hold on; stairs(t_w,u_w,'color',[0.8500 0.3250 0.0980]); stairs(t_b,u_b,'color',[0.8500 0.3250 0.0980])
            hold on; plot([0,t_sol(end)],[maxT,maxT],'k:',[0,t_sol(end)],[-maxT,-maxT],'k:',[0,t_sol(end)],[KwMaxT,KwMaxT],'k:')
             hold on; plot([0,t_sol(end)],[0,0],'k:')
            ax = gca; ax.TitleHorizontalAlignment = 'left';
            text(t_sol(end)/20,KwMaxT,'K_wT_m_a_x')
            text(t_sol(end)/20,maxT,'T_m_a_x')
            text(t_sol(end)/20,-maxT,'T_m_a_x')
            ylabel('Torque (Nm)')
        % Skip iff there is no valid solution    
        else
            disp("    " + string(thread) + ". Skipping " + string(files(thread).name))
        end
    elseif strcmp(mode,'slowstep')
        % Plot iff there is a valid solution
        if formatted_solution.solved
            % Gather all the data
            [t_sol,x_sol,u_sol,t_w,x_w,u_w,t_b,x_b,u_b] = hybrid_sim(formatted_solution);
            [maxT, mb, Ibcom, mw, Iw, l, Ib, g, a1, a2] = get_properties();
            xi = formatted_solution.input.xi;
            xf = formatted_solution.input.xf;
            % CoAM and CoE based swing-up wheel ang vel
            ww_swing = sqrt(2*l*g*(mb+mw)*(1-cos(xi(1)))*(a1+Iw)/(Iw*Iw));
            % Impact velocity for no input
            impact_noT = sqrt((2*(1-cos(xf(1)))*l*g*(mb+mw))/a1);
            maxT = formatted_solution.input.maxT;
            KwMaxT = maxT*formatted_solution.input.Kw;
            
            disp("    " + string(thread) + ". Plotting " + string(files(thread).name))
            % Create Figure
            figure('Position', [figx figy figb figh],'Name',files(thread).name,'NumberTitle','off')
            tiles = tiledlayout(5,1,'TileSpacing','compact','Padding','compact');
            %title(tiles,string(mode)+": "+string(formatted_solution.input.objective),'FontWeight','bold')
            xlabel(tiles,'Time (s)')
            sb = formatted_solution.step_brake;
             % Body Angle
            nexttile; plot([t_sol(1:sb),t_sol(sb:end)],[x_sol(1:sb,1);x_sol(sb+1:end,1)],'Marker','.','color',[0 0.4470 0.7410]); title('Body Angle')
            hold on; plot(t_w,x_w(:,1),t_b,x_b(:,1),'color',[0.8500 0.3250 0.0980])
            hold on; plot([0,t_sol(end)],[0,0],'k:',[0,t_sol(end)],[xf(1),xf(1)],'k:')
            ax = gca; ax.TitleHorizontalAlignment = 'left';
            legend('Solution','Simulation','Location','southeast')
            text(t_sol(end)/20,xf(1),'ground')
            text(t_sol(end)/20,xi(1),'ground')
            text(t_sol(end)/20,0,'equilibrium')
            ylabel('Angle (rad)')
            % Body Angular Velocity
            nexttile; plot([t_sol(1:sb),t_sol(sb:end)],[x_sol(1:sb,2);x_sol(sb+1:end,2)],'Marker','.','color',[0 0.4470 0.7410]); title('Body Angular Velocity')
            hold on; plot([t_w;t_b],[x_w(:,2);x_b(:,2)],'color',[0.8500 0.3250 0.0980])
            hold on; plot([0,t_sol(end)],[0,0],'k:')
            hold on; plot([0,t_sol(end)],[xf(2),xf(2)],'k:')
            hold on; plot([0,t_sol(end)],[impact_noT,impact_noT],'k:')
            ax = gca; ax.TitleHorizontalAlignment = 'left';
            text(t_sol(end)/20,xf(2),'\omega_m_a_x')
            text(t_sol(end)/20,impact_noT,'\omega_f_a_l_l')
            ylabel('Angular Velocity (rad/s)')
            % Wheel Angle
            nexttile; plot(t_sol(1:sb),x_sol(1:sb,3),t_sol(sb:end),x_sol(sb+1:end,3),'Marker','.','color',[0 0.4470 0.7410]); title('Wheel Angle')
            hold on; plot(t_w,x_w(:,3),t_b,x_b(:,3),'color',[0.8500 0.3250 0.0980])
            ax = gca; ax.TitleHorizontalAlignment = 'left';
            ylabel('Angle (rad)')
            % Wheel Angular Velocity
            nexttile; plot([t_sol(1:sb),t_sol(sb:end)],[x_sol(1:sb,4);x_sol(sb+1:end,4)],'Marker','.','color',[0 0.4470 0.7410]); title('Wheel Angular Velocity')
            hold on; plot([t_w;t_b],[x_w(:,4);x_b(:,4)],'color',[0.8500 0.3250 0.0980])
            hold on; plot([0,t_sol(end)],[ww_swing,ww_swing],'k:')
            ax = gca; ax.TitleHorizontalAlignment = 'left';
            text(t_sol(end)/20,ww_swing,'\omega_s_w_i_n_g')
            ylabel('Angular Velocity (rad/s)')
            % Input Motor Torque
            nexttile; stairs(t_sol,u_sol,'Marker','.','color',[0 0.4470 0.7410]); title('Motor Torque')
            hold on; stairs(t_w,u_w,'color',[0.8500 0.3250 0.0980]); stairs(t_b,u_b,'color',[0.8500 0.3250 0.0980])
            hold on; plot([0,t_sol(end)],[maxT,maxT],'k:',[0,t_sol(end)],[-maxT,-maxT],'k:',[0,t_sol(end)],[KwMaxT,KwMaxT],'k:')
            hold on; plot([0,t_sol(end)],[0,0],'k:')
            ax = gca; ax.TitleHorizontalAlignment = 'left';
            text(t_sol(end)/20,KwMaxT,'K_wT_m_a_x')
            text(t_sol(end)/20,maxT,'T_m_a_x')
            text(t_sol(end)/20,-maxT,'T_m_a_x')
            ylabel('Torque (Nm)')
        % Skip iff there is no valid solution      
        else
            disp("    " + string(thread) + ". Skipping " + string(files(thread).name))
        end
    elseif strcmp(mode,'slowfall')
        % Plot iff there is a valid solution
        if formatted_solution.solved
            % Gather all the data
            [t_sol,x_sol,u_sol,t_b,x_b,u_b] = continuous_sim(formatted_solution);
            [maxT, mb, Ibcom, mw, Iw, l, Ib, g, a1, a2] = get_properties();
            xf = formatted_solution.input.xf;
            maxT = formatted_solution.input.maxT;
            % Impact velocity for no input
            impact_noT = sqrt((2*(1-cos(xf(1)))*l*g*(mb+mw))/a1);        
            
            disp("    " + string(thread) + ". Plotting " + string(files(thread).name))
            % Create Figure
            figure('Position', [figx figy figb figh],'Name',files(thread).name,'NumberTitle','off')
            tiles = tiledlayout(5,1,'TileSpacing','compact','Padding','compact');
            %title(tiles,string(mode)+": "+string(formatted_solution.input.objective),'FontWeight','bold')
            xlabel(tiles,'Time (s)')
             % Body Angle
            nexttile; plot(t_sol,x_sol(:,1),'Marker','.','color',[0 0.4470 0.7410]); title('Body Angle')
            hold on; plot(t_b,x_b(:,1),'color',[0.8500 0.3250 0.0980])
            hold on; plot([0,t_sol(end)],[0,0],'k:',[0,t_sol(end)],[xf(1),xf(1)],'k:')
            ax = gca; ax.TitleHorizontalAlignment = 'left';
            legend('Solution','Simulation','Location','southeast')
            text(t_sol(end)/20,xf(1),'ground')
            text(t_sol(end)/20,0,'equilibrium')
            ylabel('Angle (rad)')
            % Body Angular Velocity
            nexttile; plot(t_sol,x_sol(:,2),'Marker','.','color',[0 0.4470 0.7410]); title('Body Angular Velocity')
            hold on; plot(t_b,x_b(:,2),'color',[0.8500 0.3250 0.0980])
            hold on; plot([0,t_sol(end)],[0,0],'k:')
            hold on; plot([0,t_sol(end)],[xf(2),xf(2)],'k:')
            hold on; plot([0,t_sol(end)],[impact_noT,impact_noT],'k:')
            ax = gca; ax.TitleHorizontalAlignment = 'left';
            text(t_sol(end)/20,xf(2),'\omega_m_a_x')
            text(t_sol(end)/20,impact_noT,'\omega_f_a_l_l')
            ylabel('Angular Velocity (rad/s)')
            % Wheel Angle
            nexttile; plot(t_sol,x_sol(:,3),'Marker','.','color',[0 0.4470 0.7410]); title('Wheel Angle')
            hold on; plot(t_b,x_b(:,3),'color',[0.8500 0.3250 0.0980])
            ax = gca; ax.TitleHorizontalAlignment = 'left';
            ylabel('Angle (rad)')
            % Wheel Angular Velocity
            nexttile; plot(t_sol,x_sol(:,4),'Marker','.','color',[0 0.4470 0.7410]); title('Wheel Angular Velocity')
            hold on; plot(t_b,x_b(:,4),'color',[0.8500 0.3250 0.0980])
            ax = gca; ax.TitleHorizontalAlignment = 'left';
            ylabel('Angular Velocity (rad/s)')
            % Input Motor Torque
            nexttile; stairs(t_sol,u_sol,'Marker','.','color',[0 0.4470 0.7410]); title('Motor Torque')
            hold on; stairs(t_b,u_b,'color',[0.8500 0.3250 0.0980])
            hold on; plot([0,t_sol(end)],[maxT,maxT],'k:',[0,t_sol(end)],[-maxT,-maxT],'k:')
            hold on; plot([0,t_sol(end)],[0,0],'k:')
            ax = gca; ax.TitleHorizontalAlignment = 'left';
            text(t_sol(end)/20,maxT,'T_m_a_x')
            text(t_sol(end)/20,-maxT,'T_m_a_x')
            ylabel('Torque (Nm)')
        % Skip iff there is no valid solution      
        else
            disp("    " + string(thread) + ". Skipping " + string(files(thread).name))
        end  
    end
end