close all
clear

% load matlab HW3 VAD file
load('Data_for_VAD (1).mat')

% AX=B, Sector VAD processing
% Least square method, Minimise L to find u,v
% Cost Function - L = sum(u*r(n)-u(rn))^2 minimize, take derivatie
% Radial unit vector the lidar to point on range ring - rn(cap)=sin(az)*cos(el)*x(cap) + cos(az)*cos(el)*y(cap) + sin(el)*z(cap)
% u(rn) - Radial velocity measured by the lidar

% Minimise L wrt u and v
% dl\du = 2*summation*(u*rn(cap)-u(rn))^sin(az)*cos(el)
% dl\dv = 2*summation*(u*rn(cap)-u(rn))^cos(az)*cos(el)
% dl\du=0, dl\dv=0; 2*2 Linear system, solve for u and v
% solve as AX=B 2*2 linear system


% Preallocate A, B
B=zeros(2,1);
A=zeros(2);

% 512 Data set points
T=512;


% 
for time=1:size(Data,2)
% loop over all time intervals 
%for time=1:T
    % loop for all range ring
    for ring=1:size(Data(time).range,1)

        count=0;
        % Locate ith column
        for i=1:size(Data(time).range,2)
            % test for nan if NaN dont claculate A,B matrices
             if ~isnan(Data(time).rv(ring,i))==1 && abs(Data(time).rv(ring,i))<15
                count=count+1;
                B(1)=B(1)+Data(time).rv(ring,i)*cosd(Data(time).el(ring,i))*cosd(Data(time).az(ring,i));
                B(2)=B(2)+Data(time).rv(ring,i)*cosd(Data(time).el(ring,i))*sind(Data(time).az(ring,i));
                A(1,1)=A(1,1)+cosd(Data(time).el(ring,i))*sind(Data(time).az(ring,i))*cosd(Data(time).el(ring,i))*cosd(Data(time).az(ring,i));
                A(1,2)=A(1,2)+cosd(Data(time).el(ring,i))*cosd(Data(time).az(ring,i))*cosd(Data(time).el(ring,i))*cosd(Data(time).az(ring,i));
                A(2,1)=A(2,1)+cosd(Data(time).el(ring,i))*sind(Data(time).az(ring,i))*cosd(Data(time).el(ring,i))*sind(Data(time).az(ring,i));
                A(2,2)=A(2,2)+cosd(Data(time).el(ring,i))*cosd(Data(time).az(ring,i))*cosd(Data(time).el(ring,i))*sind(Data(time).az(ring,i));
             end
        end

%%%%%%%%        % if a range ring has more than one not a NaN, do Pinv
        if count>1
            % get velocity vector (u,v) 
            output_data(time).velocity(ring,:)=pinv(A)*B;
            
            % remove velocities > 15 to reduce noise in graphs, take in
            % velocities less than 15
            if sqrt((output_data(time).velocity(ring,1))^2+(output_data(time).velocity(ring,2))^2)<15
                % Calculate resultant vector from u,v components and assign
                % it to output_data structure
                output_data(time).wind_speed(ring,1)=sqrt((output_data(time).velocity(ring,1))^2+(output_data(time).velocity(ring,2))^2);
            else 
                % if velocitites greater than 15, set it to NaN
                output_data(time).wind_speed(ring,1)=NaN;
            end
            
            % assign structure array to height to output_data, height = range * sin (el)
            output_data(time).height(ring,1)=Data(time).range(ring,1)*sind(Data(time).el(ring,1));

            %vecnorm calculatess reultant
            %wind_speed(ring,1)=vecnorm(velocity(ring,:));

            % assign output_data structure array to wind direction
            % myantan converts tan (negative angle) values to postive
            % To get linear graph over zigzag
            output_data(time).wind_direction(ring,1)=rad2deg(myatan(output_data(time).velocity(ring,2),output_data(time).velocity(ring,1)));

            % Reset A,B when did Pinc for one ring, before moving to next
            % range ring set to zero
            A=zeros(2);
            B=zeros(2,1);
        else
            % Set the radial velocties (u,v) calculated to NaN instead of
            % zero
            output_data(time).velocity(ring,:)=[NaN NaN];
            output_data(time).wind_speed(ring,1)=NaN;
            output_data(time).wind_direction(ring,1)=NaN;
            output_data(time).height(ring,1)=Data(time).range(ring,1)*sind(Data(time).el(ring,1));
            
            A=zeros(2);
            B=zeros(2,1);
        end      
    end
end 




%plot(sqrt(output_data(t).velocity(:,1).^2+output_data(t).velocity(:,2).^2),Data(t).range(:,1).*sind(Data(t).el(:,1)))

%hold on
% Preallocate 
wind_speed_mean=zeros(size(Data(1).range,1),1);
wind_direction_mean=zeros(size(Data(1).range,1),1);
% notnancount
notnancount=zeros(size(Data(time).range,1),1);

% loop for all rings
for ring=1:size(Data(time).range,1)
   
    % loop for all time
    for time=1:T
        
        % Check if NaN, then store in output
        if ~isnan(output_data(time).wind_speed(ring,1))==1
            % Get mean wind speed and direction from output_data
            wind_speed_mean(ring)=wind_speed_mean(ring)+output_data(time).wind_speed(ring,1);
            wind_direction_mean(ring)=wind_direction_mean(ring)+output_data(time).wind_direction(ring,1);
            % counts number of data points where wind speed value is not NaN
            notnancount(ring)=notnancount(ring)+1;        
        end
        
    end
    % Set zero calculated zero values to NaN, wind velocity unknown cant be
    % zero
    if notnancount(ring)==0 
        wind_speed_mean(ring)=NaN;
        wind_direction_mean(ring)=NaN;
    end
end

% Create movie for all wind velocities and directions
% for all 
% get figure
h = figure;
title('Wind speed profile')
xlabel('Wind speed (m/s)');
ylabel('Height (m)');
grid on
grid minor
% get axis of current figure
axis([0 30 0 2000])
ax = gca;
% acts like clf,clear fig slate
ax.NextPlot = 'replaceChildren';

loops = T;  %time
% Preallocate M loop array
M(loops) = struct('cdata',[],'colormap',[]);

h.Visible = 'off';
v = VideoWriter('speed_profile.avi');
v.FrameRate=15;
open(v);

% Captures frame lopp
for j = 1:loops
    % plot wind speed V height for all movie frames
    plot(output_data(j).wind_speed(:),output_data(j).height(:));
    date_time=Data(j).name(1:4)+"-"+Data(j).name(5:6)+"-"+ Data(j).name(7:8)+ " "+ Data(j).name(10:11)+":"+Data(j).name(12:13)+":"+Data(j).name(14:15);
    
    
    legend(date_time);
    % draws plots
    drawnow
    % get each frames
     M(j) = getframe(gcf);
    writeVideo(v,M(j));
end

%h.Visible = 'on';
% play movie
%movie(M,1,12);
close(v);



h = figure;
title('Wind direction vs Height from VAD')
xlabel('\theta (degrees)');
ylabel('Height (m)');
grid on
grid minor
% get axis of current figure
axis([0 400 0 2000])
ax = gca;
% acts like clf,clear fig slate
ax.NextPlot = 'replaceChildren';

loops = T;  %time
% Preallocate M loop array
M(loops) = struct('cdata',[],'colormap',[]);

h.Visible = 'off';
v = VideoWriter('direction_profile.avi');
v.FrameRate=15;
open(v);

% Captures frame lopp
for j = 1:loops
    % plot wind speed V height for all movie frames
    plot(output_data(j).wind_direction(:),output_data(j).height(:));
    date_time=Data(j).name(1:4)+"-"+Data(j).name(5:6)+"-"+ Data(j).name(7:8)+ " "+ Data(j).name(10:11)+":"+Data(j).name(12:13)+":"+Data(j).name(14:15);
    legend(date_time);
    % draws plots
    drawnow
    % get each frames
     M(j) = getframe(gcf);
    writeVideo(v,M(j));
end

%h.Visible = 'on';
% play movie
%movie(M,1,12);
close(v);





close all

% Calculate mean wind speed and direction profile at 20th time step 
%plot(output_data(20).wind_speed(:),Data(20).range(:,1).*sind(Data(20).el(:,1)))

figure(1)
plot(wind_speed_mean./notnancount,Data(1).range(:,1).*sind(Data(1).el(:,1)));
title('Wind speed vs Height from VAD')
xlabel('Wind speed (m/s)');
ylabel('Height (m)');
grid on
grid minor
figure(2)
plot(wind_direction_mean./notnancount,Data(1).range(:,1).*sind(Data(1).el(:,1)));
title('Wind direction vs Height from VAD')
xlabel('\theta (degrees)');
ylabel('Height (m)');
grid on
grid minor