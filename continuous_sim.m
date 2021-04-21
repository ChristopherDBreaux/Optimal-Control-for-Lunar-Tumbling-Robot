function [t_sol,x_sol,u_sol,t_b,x_b,u_b] = continuous_sim(formatted_solution)
    % This function simulates continuous functions such as slowfall

    % Parameters
    [maxT, mb, Ibcom, mw, Iw, l, Ib, g, a1, a2] = get_properties();
    % Extract solution data
    t_sol = formatted_solution.t_sol;
    x_sol = formatted_solution.x_sol;
    u_sol = formatted_solution.u_sol;
    xi = formatted_solution.input.xi;

    % Balance time interval
    interval_b = [0, t_sol(end)];
    xi_b = xi;
    % Balance ode45
    [t_b,x_b] = ode45(@(t,x) balance_odefun(t, x, t_sol, u_sol),interval_b,xi_b);
    % Generate input step function
    u_b = [];
    for i = 1:length(t_b)
       index = sum(t_b(i) >= t_sol);
       u_b = [u_b, u_sol(index)];
    end
end