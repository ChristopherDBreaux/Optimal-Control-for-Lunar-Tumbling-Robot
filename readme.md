# Installation Procedure

1.	Install MATLAB: https://www.mathworks.com/products/matlab.html
2.	Install Parallel Computing Toolbox: https://www.mathworks.com/products/parallel-computing.html
3.	Install YALMIP: https://yalmip.github.io/tutorial/installation/
4.	Install Gurobi: https://www.gurobi.com/downloads/gurobi-optimizer-eula/
5.	Download code: https://github.com/ChristopherDBreaux/Optimal-Control-for-Lunar-Tumbling-Robot
6.	Add YALMIP and Gurobi to path in MAIN.m

# Code Structure
The MAIN file handles batches of optimizations and data processing.
A YALMIP-based optimization function is defined for each maneuver.
An ODE function is defined for each dynamic model.
The hybrid simulator handles multiple ODE functions.
The continuous simulator handles one ODE function.

| File | Type | Description |
| :---        | :--- | :--- |
| MAIN.m | Script | This is the main script. Edit the list of inputs. Run “Optimizer” section to run a batch of inputs in parallel. Run “Plotter” section to select a previous batch for plotting. |
| swingup_opt.m | Optimizer | YALMIP function for swingup |
| slowfall_opt.m | Optimizer | YALMIP function for slowfall |
| slowstep_opt.m | Optimizer | YALMIP function for slowstep |
| hybrid_sim.m | Simulator | ode45 simulator for swingup and slowstep |
| continuous_sim.m | Simulator | ode45 simulator for slowfall |
| balance_odefun.m | ODE | Nonlinear FIP ode function |
| windup_odefun.m | ODE | Wind-up ode function |
| get_properties.m | function | Contains default system parameters |
