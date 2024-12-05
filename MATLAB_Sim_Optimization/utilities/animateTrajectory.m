function animateTrajectory(tspan, x, p, dt)
    keypoints_fn = casadi.Function.load('codegen/keypoints_fn.casadi');

    % Prepare plot handles
    hold on
    h_OA = plot([0],[0],'LineWidth',2);
    h_DB = plot([0],[0],'LineWidth',2);
    h_BC = plot([0],[0],'LineWidth',2);
    h_DE = plot([0],[0],'LineWidth',2);
    
    % Plot handles for COM markers
    h_m1 = plot(0, 0, 'kx', 'MarkerSize', 8, 'LineWidth', 2);
    h_m2 = plot(0, 0, 'kx', 'MarkerSize', 8, 'LineWidth', 2);
    h_m3 = plot(0, 0, 'kx', 'MarkerSize', 8, 'LineWidth', 2);
    h_m4 = plot(0, 0, 'kx', 'MarkerSize', 8, 'LineWidth', 2);
    
    xlabel('x'); ylabel('y');
    h_title = subtitle('t=0.0s');
    
    axis equal
    axis([-.5 .5 -.7 .3]);
    
    % Step through and update animation
    for i = 1:length(tspan)
        % % skip frame.
        % if mod(i, 1)
        %     continue;
        % end
        t = tspan(i);
        z = x(:,i); 
        keypoints = full(keypoints_fn(z,p));
    
        rA = keypoints(:,1); % Vector to point center of swing
        rB = keypoints(:,2); % Vector to hip attachment of swing
        rC = keypoints(:,3); % Vector to head
        rD = keypoints(:,4); % Vector to knee
        rE = keypoints(:,5); % Vector to foot

        r_m1 = keypoints(:,6);
        r_m2 = keypoints(:,7);
        r_m3 = keypoints(:,8);
        r_m4 = keypoints(:,9);

    
        set(h_title,'String',  sprintf('t=%.2f',t) ); % update title
        
        % Update link positions
        set(h_OA,'XData',[0 rA(1)]);
        set(h_OA,'YData',[0 rA(2)]);
        
        set(h_DB,'XData',[rD(1) rB(1)]);
        set(h_DB,'YData',[rD(2) rB(2)]);
        
        set(h_BC,'XData',[rB(1) rC(1)]);
        set(h_BC,'YData',[rB(2) rC(2)]);
        
        set(h_DE,'XData',[rD(1) rE(1)]);
        set(h_DE,'YData',[rD(2) rE(2)]);

        % Update COM markers
        set(h_m1, 'XData', r_m1(1), 'YData', r_m1(2));
        set(h_m2, 'XData', r_m2(1), 'YData', r_m2(2));
        set(h_m3, 'XData', r_m3(1), 'YData', r_m3(2));
        set(h_m4, 'XData', r_m4(1), 'YData', r_m4(2));
    
        pause(dt)
    end
end
    