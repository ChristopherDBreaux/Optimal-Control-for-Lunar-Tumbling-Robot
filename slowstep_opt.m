function swingup_opt(input, folder_name, thread)
    % This function performs a YALMIP optimization for the slowstep
    % maneuver and saves the data to a file in the batch folder
    
    % Maximum runtime
    maxCPUhours = 48;
    % Parameters
    [maxT, mb, Ibcom, mw, Iw, l, Ib, g, a1, a2] = get_properties();
    maxT = input.maxT;
    Kw = input.Kw;
    xi = input.xi;
    xf = input.xf;
    xmi = [1; 0; 0];
    N = input.N;
    s = input.s;
    
    % YALMIP variables for each time step
    uT = sdpvar(N,1,'full');

    x1 = sdpvar(N+1,1,'full'); % extra variable for initial state
    x2 = sdpvar(N+1,1,'full');
    x3 = sdpvar(N+1,1,'full');
    x4 = sdpvar(N+1,1,'full');

    xm1 = binvar(N,1); % binary varibles activate model constraints
    xm2 = binvar(N,1);
    xm3 = binvar(N,1);
    % Initialize constraints vector and cost function sum
    constraints = [];
    objective = 0;
    % Loop through each time step
    for k = 1:N
        % check cost function selected and apply correponding equation
        if strcmp(input.objective,string('time'))
            objective = 1;
        elseif strcmp(input.objective,string('energy'))
            objective = objective + norm(uT(k)*uT(k), 1);
        end
    
        % Dynamic Models
        windup = [x1(k+1) == x1(k); x2(k+1) == 0; x3(k+1) == x3(k) + x4(k)*s; x4(k+1) == x4(k) + uT(k)/Iw*s];

        brake = [x1(k+1) == x1(k); x2(k+1) == Iw*x4(k)/(a1+Iw); x3(k+1) == x3(k); x4(k+1) == 0];
        
        balance = [x1(k+1) == x1(k) + x2(k)*s; x2(k+1) == x2(k) + (a2*sin(x1(k))-uT(k))/(a1-Iw)*s; x3(k+1) == x3(k) + x4(k)*s; x4(k+1) == x4(k) + (a2*sin(x1(k))-uT(k)*a1/Iw)/(Iw-a1)*s];
    
        % Model selection based on binary variables
        constraints = [constraints; xm1(k)+xm2(k)+xm3(k)==1];
        constraints = [constraints; implies(xm1(k), [windup; 0<=uT(k)<=Kw*maxT])];
        constraints = [constraints; implies(xm2(k), brake)];
        constraints = [constraints; implies(xm3(k), [balance; -maxT<=uT(k)<=maxT])];
        % Prohibit windup or balance depending on if jump has occurred
        constraints = [constraints; implies(sum(xm2(1:k))==0, xm3(k)==0)];
        constraints = [constraints; implies(sum(xm2(1:k))==1, xm1(k)==0)];
    
    end

    % sum xm2 == 1 so it can only jump once
    constraints = [constraints; sum(xm2) == 1];
    % constrain initial state to input
    constraints = [constraints; x1(1)==xi(1); x2(1)==xi(2); x3(1)==xi(3); x4(1)==xi(4); xm1(1)==xmi(1); xm2(1)==xmi(2); xm3(1)==xmi(3)];
    % constrain terminal state to input
    constraints = [constraints; x1(N+1) == xf(1); x2(N+1) <= xf(2)];

    % YALMIP settings
    warning('off','all')
    options = sdpsettings('solver','bmibnb','bmibnb.maxiter',1e9,'bmibnb.maxtime',3600*maxCPUhours);
    solution = optimize(constraints, objective, options)

    
    
    % Structure and save data
    
    formatted_solution.input = input;
    formatted_solution.solution = solution;
    % Find brake time step
    step_brake = find(value(xm2)==1);
    formatted_solution.step_brake = step_brake;
    formatted_solution.t_brake = s*(step_brake-1);
    
    x1_sol = value(x1(1:end));
    x2_sol = value(x2(1:end));
    x3_sol = value(x3(1:end));
    x4_sol = value(x4(1:end));%[value(x4(1:step_brake-1));value(x4(step_brake+1:end))];
    formatted_solution.x_sol = [x1_sol, x2_sol, x3_sol, x4_sol];
    
    formatted_solution.ww_brake = x4_sol(step_brake);
    formatted_solution.wb_brake = x2_sol(step_brake+1);
    
    t_sol = 0:s:(N-1)*s; %t(1:end-1);
    formatted_solution.t_sol = t_sol;
    
    % Omit input during brake step because it's not an actual time step
    formatted_solution.u_sol = [value(uT(1:step_brake-1)); value(uT(step_brake+1:end)); value(uT(end))];
    
    formatted_solution.sim_time = t_sol(end);
    formatted_solution.cpu_time = duration(0,0,solution.solvertime);
    
    % Valid solution message and binary
    if contains(string(solution.info),"Successfully")
        solved = 1;
        info = "_solved_";
    else
        solved = 0;
        info = "_failed_";
    end
    formatted_solution.solved = solved;
    
    % Save to file
    file_name = folder_name + "/slowstep_" + string(input.objective) + info + "N" + string(N) + "_s" + string(s) + "_xi" + strjoin(string(xi),"_") + "_thread" + string(thread) + ".mat";
    save(file_name, 'formatted_solution')

end