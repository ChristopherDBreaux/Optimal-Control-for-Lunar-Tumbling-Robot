function dxdt = balance_odefun(t, x, t_sol, u_sol)
    % This ode function describes the FIP dynamics

    % Parameters
    [maxT, mb, Ibcom, mw, Iw, l, Ib, g, a1, a2] = get_properties();
    
    % Current t falls between two solution time steps
    index = sum(t >= t_sol);
    % Apply input of most recent time step like a step function
    u = u_sol(index);
    
    % ode
    dxdt = zeros(4,1);
	dxdt(1) = x(2);
	dxdt(2) = (a2*sin(x(1))-u)/(a1-Iw);
	dxdt(3) = x(4);
	dxdt(4) = (a2*sin(x(1))-u*a1/Iw)/(Iw-a1);
end