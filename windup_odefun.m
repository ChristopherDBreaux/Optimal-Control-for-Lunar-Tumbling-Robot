function dxdt = windup_odefun(t, x, t_sol, u_sol)
    % This ode function describes the FIP dynamics

    % Parameters
    [maxT, mb, Ibcom, mw, Iw, l, Ib, g, a1, a2] = get_properties();
    
    % Current t falls between two solution time steps
    index = sum(t >= t_sol);
    % Apply input of most recent time step like a step function
    u = u_sol(index);

    % ode
    dxdt = zeros(4,1);
	dxdt(1) = 0;
	dxdt(2) = 0;
	dxdt(3) = x(4);
	dxdt(4) = u/Iw;
end