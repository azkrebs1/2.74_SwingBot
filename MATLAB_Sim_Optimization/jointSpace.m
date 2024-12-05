function output_data = jointSpace()
    q_coeffs = load('coeffs_q.mat');
    q2_coeffs = q_coeffs.coeffs_q2;
    q3_coeffs = q_coeffs.coeffs_q3;
    q2_coeffs_notflat = q_coeffs.coeffs_q2_notflat;
    q3_coeffs_notflat = q_coeffs.coeffs_q3;
    
    dq_coeffs = load('coeffs_dq.mat');
    dq2_coeffs = dq_coeffs.coeffs_dq2;
    dq3_coeffs = dq_coeffs.coeffs_dq3;
    dq2_coeffs_notflat = dq_coeffs.coeffs_dq2;
    dq3_coeffs_notflat = dq_coeffs.coeffs_dq3;

    tau_coeffs = load('coeffs_tau.mat');
    t2_coeffs = tau_coeffs.coeffs_u2;
    t3_coeffs = tau_coeffs.coeffs_u3;
    poly_foot = [q2_coeffs, dq2_coeffs, q3_coeffs, dq3_coeffs];
    %, t2_coeffs, t3_coeffs]; 


    % % Plot the interpolated functions
    % figure(7);
    % hold on;
    % 
    % % q2_coeffs is  coefficient matrix (Nx4, where N is the number of polynomials)
    % dt = 0.4; % time step
    % timestep = linspace(0, dt, 100);
    % for i = 1:size(q2_coeffs_notflat, 1)
    %     % adjust time range for bc spline interpolation -> a_0 *
    %     (t-t_i)^3 + a_1 * (t-t_i)^2 +...
    %     time_range = (i-1) * dt + timestep;
    %     current_coeffs = q2_coeffs_notflat(i, :);
    %     poly_values = polyval(current_coeffs, timestep);
    % 
    %     plot(time_range, poly_values, 'DisplayName', sprintf('Poly %d', i));
    % end


    traj_time = 2.4;
    handles = setup_plots(); 
    % Initial leg angles for encoder resets (negative of q1,q2 in lab handout due to direction motors are mounted)
    angle1_init = pi/2; angle2_init = -pi/2; 

    % Total experiment time is buffer,trajectory,buffer
    start_period = 0; end_period  = 0;

    % Gains for impedance controller
    K_1 = 1; K_2 = 1; D_1 = 0.1; D_2 = 0.05;

    % Maximum duty cycle commanded by controller (should always be <=1.0)
    duty_max1   =  0.4;
    duty_max2   = 0.4;
    % Gains for low-level current controller
    Kp = 4; 
    Ki = 0.4; 
    num_steps = 3; 

    frdm_ip  = '192.168.1.100';     % FRDM board ip
    frdm_port= 11223;               % FRDM board port  
    params.callback = @callback; % callback function
    params.timeout  = (start_period+traj_time+end_period);  

    input = [start_period traj_time end_period angle1_init angle2_init K_1 K_2 D_1 D_2 ...
                    Kp Ki duty_max1 duty_max2 poly_foot num_steps];
    size(input)
    output_size = 15; 
    output_data = RunExperiment(frdm_ip,frdm_port,input,output_size,params);


    function callback(new_data)
        % Parse new data
        t = new_data(:,1);          % time
        th1 = new_data(:,2);       % position
        dth1 = new_data(:,3);       % velocity
        cur1 = new_data(:,4);       % current
        dcur1 = new_data(:,5);      % desired current
        duty1 = new_data(:,6);      % command

        th2 = new_data(:,7);       % position
        dth2 = new_data(:,8);       % velocity
        cur2 = new_data(:,9);       % current
        dcur2 = new_data(:,10);     % desired current
        duty2 = new_data(:,11);     % command

        th1_des = new_data(:, 12); 
        dth1_des = new_data(:, 13); 
        th2_des = new_data(:, 14); 
        dth2_des = new_data(:, 15); 

        N = length(th1);

        % Create arrays of handles and data
        handlesMotor1 = [handles.h1, handles.h2, handles.h3, handles.h4, handles.h5, handles.h6, handles.h7];
        handlesMotor2 = [handles.h8, handles.h9, handles.h10, handles.h11, handles.h12, handles.h13, handles.h14];
        dataMotor1 = {th1, th1_des, dth1, dth1_des, cur1, dcur1, duty1, th1_des, dth1_des};
        dataMotor2 = {th2, th2_des, dth2, dth2_des, cur2, dcur2, duty2, th2_des, dth2_des};

        % Update Motor 1 plots
        for i = 1:length(handlesMotor1)
            handlesMotor1(i).XData(end+1:end+N) = t;   
            handlesMotor1(i).YData(end+1:end+N) = dataMotor1{i}; % Use appropriate data variable
        end

        % Update Motor 2 plots
        for i = 1:length(handlesMotor2)
            handlesMotor2(i).XData(end+1:end+N) = t;   
            handlesMotor2(i).YData(end+1:end+N) = dataMotor2{i}; % Use appropriate data variable
        end

    end
    function handles = setup_plots()
        % Initialize output structures
        handles = struct();
        axes = struct();



        % Motor 1
        figure(2); clf; hold on; % Clear figure

        % Subplot 1: Angle 1
        axes.a1 = subplot(421);
        handles.h1 = plot(0, 0, 'b'); % Angle 1 (Motor 1)
        handles.h1.XData = []; handles.h1.YData = [];
        ylabel('Angle 1 (rad)');
        hold on;
        handles.h2 = plot(0, 0, 'b--'); % Angle 1 (Motor 2)
        handles.h2.XData = []; handles.h2.YData = [];

        % Subplot 5: Angle 2 (Motor 2)
        axes.a5 = subplot(421);
        handles.h8 = plot(0, 0, 'k'); % Angle 2 (Motor 2)
        handles.h8.XData = []; handles.h8.YData = [];
        ylabel('Angle 2 (rad)');
        handles.h9 = plot(0, 0, 'k--'); % Angle 2 (Motor 2 - another line)
        handles.h9.XData = []; handles.h9.YData = [];
        hold off;

        % Subplot 2: Velocity 1
        axes.a2 = subplot(422);
        handles.h3 = plot(0, 0, 'b'); % Velocity 1 (Motor 1)
        handles.h3.XData = []; handles.h3.YData = [];
        ylabel('Velocity 1 (rad/s)');
        hold on;
        handles.h4 = plot(0, 0, 'r--'); % Velocity 1 (Motor 2)
        handles.h4.XData = []; handles.h4.YData = [];
        hold off;

        % Subplot 3: Current 1
        axes.a3 = subplot(423);
        handles.h5 = plot(0, 0, 'r'); % Current 1 (Motor 1)
        handles.h5.XData = []; handles.h5.YData = [];
        ylabel('Current 1 (A)');
        hold on;
        handles.h6 = plot(0, 0, 'r--'); % Current 1 (Motor 2)
        handles.h6.XData = []; handles.h6.YData = [];
        hold off;

        % Subplot 4: Duty Cycle 1
        axes.a4 = subplot(426);
        handles.h7 = plot(0, 0, 'b'); % Duty Cycle 1 (Motor 1)
        handles.h7.XData = []; handles.h7.YData = [];
        ylabel('Duty Cycle 1');

        % Motor 2 (same figure, additional handles)
        figure(2); % Continuing in figure(1)

        % Subplot 5: Angle 2 (Motor 2)
        axes.a5 = subplot(425);
        handles.h8 = plot(0, 0, 'k'); % Angle 2 (Motor 2)
        handles.h8.XData = []; handles.h8.YData = [];
        ylabel('Angle 2 (rad)');
        hold on;
        handles.h9 = plot(0, 0, 'k--'); % Angle 2 (Motor 2 - another line)
        handles.h9.XData = []; handles.h9.YData = [];
        hold off;

        % Subplot 6: Velocity 2 (Motor 2)
        axes.a6 = subplot(424);
        handles.h10 = plot(0, 0, 'g'); % Velocity 2 (Motor 2)
        handles.h10.XData = []; handles.h10.YData = [];
        ylabel('Velocity 2 (rad/s)');
        hold on;
        handles.h11 = plot(0, 0, 'k--'); % Velocity 2 (Motor 2 - another line)
        handles.h11.XData = []; handles.h11.YData = [];
        hold off;

        % Subplot 7: Current 2 (Motor 2)
        axes.a7 = subplot(427);
        handles.h12 = plot(0, 0, 'g'); % Current 2 (Motor 2)
        handles.h12.XData = []; handles.h12.YData = [];
        ylabel('Current 2 (A)');
        hold on;
        handles.h13 = plot(0, 0, 'k--'); % Current 2 (Motor 2 - another line)
        handles.h13.XData = []; handles.h13.YData = [];
        hold off;

        % Subplot 8: Duty Cycle 2 (Motor 2)
        axes.a8 = subplot(428);
        handles.h14 = plot(0, 0, 'g'); % Du
        ylabel('Duty Cycle 2');
    end
end

