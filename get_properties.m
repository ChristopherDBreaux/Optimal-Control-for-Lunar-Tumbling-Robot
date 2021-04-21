function [maxT, mb, Ibcom, mw, Iw, l, Ib, g, a1, a2] = get_properties()
    % This function defines the robot properties used by the optimizers and
    % simulators

    maxT = 0.134;   % max motor torque 0.134
    mb = 5;         % body mass
    Ibcom = 0.05;      % body inertia about COM
    mw = 0.75;      % flywheel mass
    Iw = 0.002;     % flywheel inertia about COM
    l = 0.18;       % fulcrum to body COM = fulcrom to wheel COM
    Ib = Ibcom + mb*l*l;    % body inertia about fulcrum
    g = 1.62;       % moon gravity
    a1 = mw*l*l+Ib;
    a2 = (mb*l+mw*l)*g;
end