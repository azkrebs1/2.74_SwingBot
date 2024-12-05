%% Test Simulation Environment
setupSim();

assert(isfile('codegen/A_fn.casadi'), 'A_fn.casadi file does not exist in the specified path.');

A_fn = casadi.Function.load('codegen/A_fn.casadi');
b_fn = casadi.Function.load('codegen/b_fn.casadi');


%% [SIMULATE]:
dt_sim = 0.001;
% t_sim = 0:dt_sim:t_span(end);
t_sim = linspace(0, 10, 10000);
N_sim = length(t_sim);

z_0 = [0; pi/2; pi/2; 0; 0; 0];
z_sim = zeros(6, N_sim);
z_sim(:,1) = z_0;
for i = 1:N_sim-1
    dz = dynamics(z_sim(:,i), params, A_fn, b_fn);
    z_sim(4:6, i+1) = z_sim(4:6, i) + dz(4:6)*dt_sim;
    z_sim(1:3, i+1) = z_sim(1:3,i) + z_sim(4:6,i+1)*dt_sim;
end


figure(1); clf; hold on;
title("Simulation animation");
animateTrajectory(t_sim, z_sim, params, dt_sim);


%% Dynamics
function dz = dynamics(z,p, A_fn, b_fn)
    A = A_fn(z,p);
    u = [0 0]';
    b = b_fn(z, u, p);
    qdd = full(A\b);
    dz = 0*z;
    dz(1:3) = z(4:6);
    dz(4:6) = qdd;
end




