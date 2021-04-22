function slowfall_opt(input, folder_name, thread)
    % This function performs a YALMIP optimization for the slowfall
    % maneuver and saves the data to a file in the batch folder

    % Maximum runtime
    maxCPUhours = 48;
    % Parameters
    [maxT, mb, Ibcom, mw, Iw, l, Ib, g, a1, a2] = get_properties();
    maxT = input.maxT;
    xi = input.xi;
    xf = input.xf;
    N = input.N;
    s = input.s;
    
    % YALMIP variables for each time step
    uT = sdpvar(N,1,'full');

    x1 = sdpvar(N+1,1,'full'); % extra variable for initial state
    x2 = sdpvar(N+1,1,'full');
    x3 = sdpvar(N+1,1,'full');
    x4 = sdpvar(N+1,1,'full');
    % Initialize constraints vector and cost function sum
    constraints = [];
    objective = 0;
    % Loop through each time step
    for k = 1:N
        % check cost function selected and apply correponding equation
        if strcmp(input.objective,string('time'))
            objective = 1;
        elseif strcmp(input.objective,string('energy'))
            objective = objective + abs(uT(k)) %norm(uT(k)*uT(k), 1);
        end

        % Dynamic Model
        constraints = [constraints; x1(k+1) == x1(k) + x2(k)*s; x2(k+1) == x2(k) + (a2*sin(x1(k))-uT(k))/(a1-Iw)*s; x3(k+1) == x3(k) + x4(k)*s; x4(k+1) == x4(k) + (a2*sin(x1(k))-uT(k)*a1/Iw)/(Iw-a1)*s];
        constraints = [constraints; -maxT<=uT(k)<=maxT];
        constraints = [constraints; x1(k+1)>=x1(k)];
    end

    % constrain initial state to input
    constraints = [constraints; x1(1)==xi(1); x2(1)==xi(2); x3(1)==xi(3); x4(1)==xi(4)];
    % constrain terminal state to input
    constraints = [constraints; x1(N+1) == xf(1); x2(N+1) <= xf(2)];

    % YALMIP settings
    warning('off','all')
    options = sdpsettings('solver','bmibnb','bmibnb.maxiter',1e9,'bmibnb.maxtime',3600*maxCPUhours);
    solution = optimize(constraints, objective, options)

    
    
    
    % Structure and save data
    
    formatted_solution.input = input;
    formatted_solution.solution = solution;
    
    x1_sol = value(x1(1:end));
    x2_sol = value(x2(1:end));
    x3_sol = value(x3(1:end));
    x4_sol = value(x4(1:end));
    formatted_solution.x_sol = [x1_sol, x2_sol, x3_sol, x4_sol];
    
    t_sol = 0:s:(N)*s;
    formatted_solution.t_sol = t_sol;
    
    formatted_solution.u_sol = [value(uT(1:end));value(uT(end))];
    
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
    file_name = folder_name + "/slowfall_" + string(input.objective) + info + "N" + string(N) + "_s" + string(s) + "_xi" + strjoin(string(xi),"_") + "_thread" + string(thread) + ".mat";
    save(file_name, 'formatted_solution')
    
end
    