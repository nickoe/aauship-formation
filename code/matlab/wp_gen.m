function [ heading ] = wp_gen( wps, wpe, now, speed)
%WP_GEN Waypoint Generator
%   This calculates new sub-waypoints and headings there to 

    vessel = [now 0]; % [x y angle]
    wp_radius = 10;
    intermediate_length = 1;
    n = 1;

% %     h = figure(1);
% %     hold on
% %     axis equal


    %% Begnning of calculations
% %     plot(vessel(1),vessel(2),'r*') % Initial point of vessel
    p_pos = vessel;
    vessel_p = p_pos(1:2);

    % Put the vessels position in a the first point of the track to make
    % the vessel reach the first waypoint in the track.
%     track = [p_pos(1:2);track];
    track = [wps;wpe];

    %% Track frame projected point
    intermediate_vector = (track(n+1,:)-track(n,:))/norm(track(n+1,:)-track(n,:))*intermediate_length;
    vessel = p_pos;
    vessel_p = [vessel(1)+intermediate_vector(1) vessel(2)+intermediate_vector(2)]; % Intermediate forward parallel to track projection vessel point
% %     plot(vessel_p(1),vessel_p(2),'g*') % Plot of intermediate point

    %% Calculate the projected point onto the track
    A = vessel_p-track(n,:);
    B = track(n+1,:)-track(n,:);
    P = ((dot(A,B))/((norm(B)^2)))*B; % Projection of vector on vector
    P = P + track(n,:);
% %     plot(P(1),P(2),'k*')

    %% The vessels predicted position
    p_pos = P - vessel(1:2);
    p_pos = p_pos/norm(p_pos);% Normalize vector
    p_pos = p_pos*speed+vessel(1:2); % Simulate speed
% %     plot(p_pos(1),p_pos(2),'rx')
    % vesselp(1,1,1,1,1)

    %% Calculate if waypoint is reached
%     dist = sqrt((p_pos(1)-track(n+1,1))^2+(p_pos(2)-track(n+1,2))^2);
%     if dist < wp_radius
%         disp('hej')
%     end
    
%     A = vessel;  % Das schiff
%     B = p_pos;   % Das punkt
%     heading = 2*pi-asin( (B(2)-A(2)) / sqrt( (B(1)-A(1))^2 + (B(2)-A(2))^2) );
    heading = 2*pi-asin( (p_pos(1)-vessel(1)) / sqrt( (p_pos(1)-vessel(1))^2 + (p_pos(2)-vessel(2))^2) );

end