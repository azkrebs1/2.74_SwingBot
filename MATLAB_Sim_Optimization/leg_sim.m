%% 2.74 Leg Sim and Optimization
setupSim();

%% Trajectory optimization with CasADi
% CasADi is a symbolic framework for algorithmic differentiation and numerical optimization.
% It's what our lab uses extensively to design controllers for locomotion, jumps, backflips, etc.
 
% The syntax is similar to MATLAB's symbolic toolbox but it's much more 
% powerful and expressive for us to formulate optimizations.

% Access to the following functions
% A_fn(z, param)
% b_fn(z, u, param)
% energy_fn(z, param)
% pos_end_effector(z, param)
% vel_end_effector(z, param)
% J_end_effector(z, param)
% keypoints_fn(z, param)

%% [DECISION VARIABLES AND PARAMETERS]:
q_dim = 3;                  % Number of generalized coordinates
N = 22;                     % Size of optimization horizon
dt = .4;                    % Discretization time step
t_span = 0:dt:(N - 1)*dt;   % Time span

opti = casadi.Opti();

q = opti.variable(3, N);
q_dot = opti.variable(3, N);
u = opti.variable(2, N-1);

p = opti.parameter(19, 1);

% Add additional parameters for the initial conditions + state constraints + input constraints.
% Use "inf" or "-inf" if a state does not need to be constrained.
q_0 = opti.parameter(3, 1);
q_dot_0 = opti.parameter(3, 1);
q_max = opti.parameter(3, 1); % Rad - could potentially make this 2, 1 for no max or min on q1
q_min = opti.parameter(3, 1); % Rad
tau_max = opti.parameter(2, 1); % Nm
tau_min = opti.parameter(2, 1); % Nm


%% [CONSTRAINTS]:
opti.subject_to(q(:, 1) == q_0);
opti.subject_to(q_dot(:, 1) == q_dot_0);

for k = 1:N - 1
    % Semi-implicit Euler integration 
    z_k = [q(:, k); q_dot(:, k)];
    u_k = u(:, k);
    qdd_k = A_fn(z_k, p) \ (b_fn(z_k, u_k, p));     % qdd = (M_inv) * (tau - C - G)
    opti.subject_to(q_dot(:, k + 1) == q_dot(:, k) + qdd_k * dt);
    opti.subject_to(q(:, k + 1) == q(:, k) + q_dot(:, k + 1) * dt);

    % Min/max state constraints
    opti.subject_to(q(:, k) <= q_max);
    opti.subject_to(q(:, k) >= q_min);

    opti.subject_to(u(:, k) <= tau_max);
    opti.subject_to(u(:, k) >= tau_min);

end


%% [OBJECTIVE]:
% Cost should be SCALAR - check size(cost)
% Torque minimization cost
cost_torque = 0; 
for k = 1:N - 1
    cost_torque = cost_torque + u(:, k)' * u(:, k);
end

cost_height = q(1,end);

% Weights for each cost 
Q_torque = 0;
Q_height = 1;

cost = Q_torque*cost_torque + Q_height * cost_height;
opti.minimize(cost);

%% [SOLVER]:
opti.solver('ipopt');

%% [SET PARAMETER VALUES]:
z_0 = [0; pi/2; pi/2; 0; 0; 0];
opti.set_value(q_0, z_0(1:3));
opti.set_value(q_dot_0, z_0(4:6));
opti.set_value(p, params);
opti.set_value(q_max, q_max_val);
opti.set_value(q_min, q_min_val);
opti.set_value(tau_max, tau_max_val);
opti.set_value(tau_min, tau_min_val);


%% [SOLVE]:
soln = opti.solve();
q_soln = soln.value(q);
q_dot_soln = soln.value(q_dot);
z_soln = [q_soln; q_dot_soln];
u_soln = soln.value(u);

%% [SIMULATE]:
dt_sim = 0.001;
t_sim = 0:dt_sim:t_span(end);
N_sim = length(t_sim);

% Interpolation schemes: {'nearest', 'linear', 'spline', 'pchip', 'cubic'}
[u_out, coeffs_u2, coeffs_u3] = interpolateOptimizedControl(t_span, u_soln, t_sim, 'spline');
[q_out, coeffs_q2, coeffs_q3] = interpolateOptimizedControl(t_span, q_soln(2:3,2:end), t_sim, 'spline');
[dq_out, coeffs_dq2, coeffs_dq3] = interpolateOptimizedControl(t_span, q_dot_soln(2:3,2:end), t_sim, 'spline');

% Flatten coeffs_u2 and coeffs_u3
coeffs_u2_flat = reshape(coeffs_u2', 1, []); % Transpose to get rows in sequence, then flatten
coeffs_u3_flat = reshape(coeffs_u3', 1, []);

% Flatten coeffs_q2 and coeffs_q3
coeffs_q2_flat = reshape(coeffs_q2', 1, []);
coeffs_q3_flat = reshape(coeffs_q3', 1, []);

% Flatten coeffs_dq2 and coeffs_dq3
coeffs_dq2_flat = reshape(coeffs_dq2', 1, []);
coeffs_dq3_flat = reshape(coeffs_dq3', 1, []);


% Save flattened arrays into coeffs_tau.mat
coeffs_tau.coeffs_u2 = coeffs_u2_flat;
coeffs_tau.coeffs_u3 = coeffs_u3_flat;
coeffs_tau.coeffs_u2_notflat = coeffs_u2;
coeffs_tau.coeffs_u3_notflat = coeffs_u3;
save('coeffs_tau.mat', '-struct', 'coeffs_tau');

% Save flattened arrays into coeffs_q.mat
coeffs_q.coeffs_q2 = coeffs_q2_flat;
coeffs_q.coeffs_q3 = coeffs_q3_flat;
coeffs_q.coeffs_q2_notflat = coeffs_q2;
coeffs_q.coeffs_q3_notflat = coeffs_q3;
save('coeffs_q.mat', '-struct', 'coeffs_q');

% Save flattened arrays into coeffs_dq.mat
coeffs_dq.coeffs_dq2 = coeffs_dq2_flat;
coeffs_dq.coeffs_dq3 = coeffs_dq3_flat;
coeffs_dq.coeffs_dq2_notflat = coeffs_dq2;
coeffs_dq.coeffs_dq3_notflat = coeffs_dq3;
save('coeffs_dq.mat', '-struct', 'coeffs_dq');

u_out(:, end - floor(dt/dt_sim):end) = 0;
z_sim = zeros(6, N_sim);
z_sim(:,1) = z_0;

i_sim = zeros(1, N_sim);

% Gains for PD control
Kp = diag([10, 10, 10]); 
Kd = diag([2, 2, 2]);
for i = 1:N_sim-1
    q_des = interp1(t_span, q_soln', t_sim(i))';
    q_dot_des = interp1(t_span, q_dot_soln', t_sim(i))';

    % feedforward torque for simulation
    u_fb = Kp * (q_des - z_sim(1:3, i)) + Kd * (q_dot_des - z_sim(4:6, i));
    u_ff = interp1(t_span(1:end-1), u_soln', t_sim(i), 'previous')';  % feedforward control
    u_total = u_fb(2:3);

    % Dynamics 
    A = full(A_fn(z_sim(:, i), params));
    b = full(b_fn(z_sim(:, i), u_total, params));
    % qdd = A \ b;
    qdd = pinv(A) * b; % done so no singularity in the matrix

    % Update state
    z_sim(4:6, i+1) = z_sim(4:6, i) + qdd * dt_sim;
    z_sim(1:3, i+1) = z_sim(1:3, i) + z_sim(4:6, i+1) * dt_sim;
end

%% [PLOTS]:
max_constraint = pi * ones(size(t_span));
min_constraint = pi/6 * ones(size(t_span));

figure(1); clf; hold on;
title("Joint trajectory");
plot(t_span(1:end-1), q_soln(1, 1:end-1), 'r--');
plot(t_span(1:end-1), q_soln(2, 1:end-1), 'b--');
plot(t_span(1:end-1), q_soln(3, 1:end-1), 'k--');
plot(t_sim(1:end-dt*1000), z_sim(1, 1:end-dt*1000), 'r-');
plot(t_sim(1:end-dt*1000), z_sim(2, 1:end-dt*1000), 'b-');
plot(t_sim(1:end-dt*1000), z_sim(3, 1:end-dt*1000), 'k-');
legend('q_1_{opt}', 'q_2_{opt}', 'q_3_{opt}', 'q_1_{sim}', 'q_2_{sim}', 'q_3_{sim}');
xlabel('Time (s)'); ylabel('q (rad)');

figure(2); clf; hold on;
title("Torque trajectory");
stairs(t_span(1:end-1), u_soln(1, :), 'r--');
stairs(t_span(1:end-1), u_soln(2, :), 'b--');
stairs(t_sim, u_out(1, :), 'r-');
stairs(t_sim, u_out(2, :), 'b-');
legend('\tau_1_{opt}', '\tau_2_{opt}', '\tau_1_{sim}', '\tau_2_{sim}');
xlabel('Time (s)'); ylabel('\tau (Nm)');

figure(3); clf; hold on;
title("Optimization animation");
animateTrajectory(t_span, z_soln, params, dt);

figure(4); clf; hold on;
title("Simulation animation");
animateTrajectory(t_sim(:, 1:end - dt*1000), z_sim(:, 1:end - dt*1000), params, dt_sim);

function [u_interp, coeffs_u1, coeffs_u2] = interpolateOptimizedControl(t_span, u_soln, t_sim, interp_scheme)
    u_interp1 = interp1(t_span(1:end-1), u_soln(1, :), t_sim, interp_scheme); 
    u_interp2 = interp1(t_span(1:end-1), u_soln(2, :), t_sim, interp_scheme); 
    u_interp = [u_interp1; u_interp2];
    pp_u1 = spline(t_span(1:end-1), u_soln(1, :)); 
    pp_u2 = spline(t_span(1:end-1), u_soln(2, :));
    
    coeffs_u1 = pp_u1.coefs;  % coefficients for u_soln(1, :)
    coeffs_u2 = pp_u2.coefs;  % coefficients for u_soln(2, :)
end