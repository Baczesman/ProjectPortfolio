%tested using MATLAB R2022b
%requires Control System Toolbox for unscentedKalmanFilter to run

%% loading GPS IMU and ODOM data

%opening files:
%to change files used, change names and make sure IMU_Hz is adjusted

GPS = readtable('20230309_1814_gps_data.csv');
IMU = readtable('20230309_1814_imu_data.csv');
ODOM = readtable('20230309_1814_odom_data.csv');
IMU_Hz = 5;    %is 5 or 20, depending on data
GPS_Hz = 1;     %always 1

graph_size = 30;      %size of position graph (higher value, moe zoomed out)
Rate = IMU_Hz/GPS_Hz;       %Rate will always be = IMU_Hz, 
dt = 1/Rate;    %used to calculate speed change
gps_start = 1;  %starting point in GPS data, for starting at a certain point of data
initial_state = [GPS.osx(gps_start), GPS.osy(gps_start), GPS.speed(gps_start), 180-GPS.bearing(gps_start)-90, IMU.ax(1), IMU.wz(1)]; %initial state of kalman filter
%since bearing is 0 for true north, and we are using OSGB, we need to flip
%and rotate the bearing, hence bearing parsed into the function is always
%180-GPS.bearing(i)-90 (flip, turn 90 degrees left)

%%  allocating ram to variables for storing predicted, corrected and gps data
%x position, y position (in OSGB), speed, bearing (0 is north, clockwise)
xpred = zeros(size(GPS,1)-1, 6);
xcorr = zeros(size(GPS,1)-1, 6);
xysb = zeros(size(GPS,1)-1,4);
%if using 20230405_0858_imu_data.csv, change size(GPS,1)-1 to size(GPS,1)-2,
%this is because file is missing entires


%%  creating unscented kalman filter object
% @vehicle_motion_model and @vehicle_measurement_model are funstion handles
% vehicle_motion_model is called when using .predict and
% vehicle_measurement_model is called when .correct
filter = unscentedKalmanFilter(@vehicle_motion_model, @vehicle_measurement_model, initial_state, ...
    'HasMeasurementWrapping', true, ...  %bearing is between 0 and 360, so this is included
    'MeasurementNoise', diag([1,1,1,2, 0.15, 0.15]), ...  %noise in measured values (estimated)
    'ProcessNoise', diag([0.02, 0.02, 0.4, 0.05, 0.0, 0.0]));   %process noise



%% creating and storing data

for i=gps_start:size(xcorr,1)
    for j=1:Rate  %assumes gps hz to 1 always
        [xpred(i,:), Ppred] = predict(filter, dt);  %, IMU.ax(Rate*i+j-Rate), IMU.wz(Rate*i+j-Rate)
        %predict calls the vehicle_motion_model function which is the state
        %transition function. The program generates sigma points, captures
        %the mean and covariance of the state estimate. 
        %the function is called 20 times, by then the filter object will
        %have the estimated location based on the prediction (stored at the end)
    end
    [xcorr(i,:),Pcorr] = correct(filter, [GPS.osx(i+1), GPS.osy(i+1), GPS.speed(i+1), 180-GPS.bearing(i+1)-90, IMU.ax(i+1), IMU.wz(i+1)],1);  
    %the correct function updates the state and state estimation error
    %covariance using the measured data. It uses the predicted state stored
    %in filter and covariance to update covariance and predicted state at
    %next step
    xysb(i,:) = [GPS.osx(i),GPS.osy(i), GPS.speed(i), 180-GPS.bearing(i)-90];
end
%both predict and correct generate 9  (2*states + 1) sigma points


%%  saving geneated data to a .cvs file
fid = fopen('final_results.csv','wt');
fprintf( fid, "time_offset, x, y, speed");

%fprintf( fid, '\n%f,%f,%f,%f', time_offset, x, y, speed ); save format

for i=gps_start:size(xcorr,1)
    fprintf( fid, '\n%f,%f,%f,%f', i,xcorr(i,1) , xcorr(i,2), xcorr(i,3) );
    %since all GPS data starts at t=0 and has ~1 second itervals, it's
    %assigned as i.
    %xcorr 1, 2 and 3 are x pos, y pos and speed respectively
end
fclose(fid);    %closing the save file

%% plotting entire journey - gps and corrected
clf;
hold on;
title('Position on OSGB','black - GPS, red - corrected data')
plot(xcorr(:,1), xcorr(:,2), "Color",[1,0,0], MarkerSize=8, Marker='.' );  %position after correcting
plot(xysb(:,1), xysb(:,2), "Color",[0,0,0], MarkerSize=8, Marker='.' );   %actual gps position

%%  ^ run the section to get a better view of the journey
waitforbuttonpress;
%% plotting speed
clf;
hold on;
title('Speed data','black - GPS, RED - corrected')
for i=gps_start:size(xcorr,1)
    plot(i, xcorr(i,3), "Color",[1,0,0], MarkerSize=8, Marker='.' );  %position after correcting
    plot(i, xysb(i,3), "Color",[0,0,0], MarkerSize=8, Marker='.' );   %actual gps position
end

%%  ^ run the section to get a better view of the speed
waitforbuttonpress;
%%  plotting predicted, corrected and gps location in 'animation' 'chase' style
%black - GPS data
%red - estimated state with improved accuracy
%green - estimate from imu data
clf;
hold on;
title('position','Black - GPS, Red - Corrected, green - predicted')
for i=gps_start:size(xcorr,1)
    plot(xpred(i,1), xpred(i,2), "Color",[0,1,0], MarkerSize=4, Marker='.' );   %preicted position from sensor data
    plot(xcorr(i,1), xcorr(i,2), "Color",[1,0,0], MarkerSize=8, Marker='.' );  %position after correcting
    plot(xysb(i,1), xysb(i,2), "Color",[0,0,0], MarkerSize=8, Marker='.' );   %actual gps position
    xlim([xysb(i,1)-graph_size/2,xysb(i,1)+graph_size/2]);  %center on point x
    ylim([xysb(i,2)-graph_size/2,xysb(i,2)+graph_size/2]);  %center on point y
    pause(0.2); %delay 
end

%% motion model
%the model takes in the state (pos, speed, bearing) and imu data for
%forward acceleration and turning rate. It's used to calculate the next
%state
function state = vehicle_motion_model(state, varargin)
    x_pos = state(1);
    y_pos = state(2);
    speed = state(3);
    bearing = state(4);
    ax1 = state(5);
    wz = state(6);
    dt = varargin{1};   %time 
    g = 9.81;   %9.81m/s^2 used to convert ax from g to m/s^2
    ax = (ax1)*g;   % acceleration in m.s
    x_pos = x_pos + speed * cosd(bearing) * dt + 0.5 * ax * dt^2 * cosd(bearing);  %next position equation
    y_pos = y_pos + speed * sind(bearing) * dt + 0.5 * ax * dt^2 * sind(bearing);  %next position equation
    new_speed = speed + ax * dt;   %since acceleration is in g's we need to convert it to m/s2
    new_bearing = mod(bearing + wz * dt, 360);

    state = ([x_pos, y_pos, new_speed, new_bearing, ax1, wz]);
end


%% measurement model
function [measurement, bounds] = vehicle_measurement_model(state, varargin)
    measurement = [state(1), state(2), state(3), state(4), state(5), state(6)];
    if nargout == 2
        bounds = [0 Inf;0 Inf;0 Inf;0 360; -Inf, Inf; -Inf, Inf]; %bounds for wrapping
    end
end


%% conclusion
%{
The decision to use the unscented kalman filter was because it was well
documented and works with nonlinear models, uses descrete time points (like
the one I have). It alsohas a medium computational cost, compared to the 
high cost of a particle filter.

The corrected position is essentially a smoothed out trajectory. The
original GPS readings don't have very obvious noise to start with, but on
turns we can see that the GPS undershoots compared to the IMU data, and the
corrected state is closer to what we coukd expect. On top of that one-off
measurements on straight lines are slightly less taken into account. When
the vehicle is stopped the new corrected location is also less noisy being
more 

The filter doesn't properly filter out all noise. This is mostly due to the
unideal meaurement noise and process noise vectors. Further tweaking these
values can increase the accuracy and anomaly handling of the system.
For example increasing process noise makes the system rely less on the new 
corrected position, and increasing measurement noise makes the system rely 
more on the predicted location, rather than the GPS location, but
I am happy with the current values and results.

The system could utilize wheel odomentry to also track distance, which
could further improve readings, however this would also increase the
complexity. Since the wheel odometry data doesn't have a set time offset it
would be computationally demanding just to find the right correct data.
Another issue is fusing odometry and IMU data to correct GPU accuracy. IMU
already gives us a way of more accurately predicting travelled distance, so
another reading for the same value would be hard to work with. It could be
used instead of IMU data, or compared before passing through the filter,
but I settled on just using IMU data for simplicity.


References:
The MathWorks, Inc. (2023) Extended and Unscented Kalman Filter Algorithms
for Online State Estimation,
Available online: https://uk.mathworks.com/help/control/ug/extended-and-unscented-kalman-filter-algorithms-for-online-state-estimation.html

The MathWorks, Inc. (2023) Nonlinear State Estimation Using Unscented
Kalman Filter and Particle Filter, 
Available online: https://uk.mathworks.com/help/control/ug/nonlinear-state-estimation-using-unscented-kalman-filter.html

MATLAB (2017), Nonlinear State Estimators | Understanding Kalman Filters,
Part 5 [Video] Available online: https://www.youtube.com/watch?v=Vefia3JMeHE&list=PLn8PRpmsu08pzi6EMiYnR-076Mh-q3tWr&index=5
[Accessed 28.04.2023]

Mokhamad Nur Cahyadi, Tahiyatul Asfihani, Ronny Mardiyanto, Risa Erfianti
(2022) Performance of GPS and IMU sensor fusion using unscented Kalman filter for precise i-Boat navigation in infinite wide waters,
Geodesy and Geodynamics, ISSN 1674-9847, Available online: https://doi.org/10.1016/j.geog.2022.11.005

%}
