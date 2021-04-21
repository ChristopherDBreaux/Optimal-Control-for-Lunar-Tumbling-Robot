function [t_sol,x_sol,u_sol,t_w,x_w,u_w,t_b,x_b,u_b] = hybrid_sim(formatted_solution)
    % This function simulates hybrid functions with a brake jump such as
    % swingup and slowstep

    % Parameters
    [maxT, mb, Ibcom, mw, Iw, l, Ib, g, a1, a2] = get_properties();
    % Extract solution data
    t_sol = formatted_solution.t_sol;
    x_sol = formatted_solution.x_sol;
    u_sol = formatted_solution.u_sol;
    t_brake = formatted_solution.t_brake;
    xi = formatted_solution.input.xi;
    
    % Windup time interval
    interval_w = [0, t_brake];
    xi_w = xi;
    % Windup ode45
    [t_w,x_w] = ode45(@(t,x) windup_odefun(t, x, t_sol, u_sol),interval_w,xi_w);
    
    % Generate input step function
    u_w = [];
    for i = 1:length(t_w)
        index = sum(t_w(i) >= t_sol);
       u_w = [u_w, u_sol(index)];
    end
    
    % Balance time interval
    interval_b = [t_brake, t_sol(end)];
    xi_b = zeros(4,1);
    xi_b(1) = x_w(end,1); % body position
    xi_b(2) = Iw*x_w(end,4)/(a1+Iw); % body velocity
    xi_b(3) = x_w(end,3); % wheel position
    xi_b(4) = 0; % wheel velocity
    [t_b,x_b] = ode45(@(t,x) balance_odefun(t, x, t_sol, u_sol),interval_b,xi_b);
    
    % Generate input step function
    u_b = [];
    for i = 1:length(t_b)
       index = sum(t_b(i) >= t_sol);
       u_b = [u_b, u_sol(index)];
    end
end