% Contributors: Emma Haas, Keegan Garncarz
% Course Number: ASEN 3801
% File Name: main_3.5.m
% Created On: 3/31/2026


clear; clc; close all;

%% Parameters
g  = 9.81;        
Ix = 5.8e-5;      
Iy = 7.2e-5;      

%% Inner-loop gains 

K1_lat = 12*Ix;
K2_lat = 20*Ix;

K1_lon = 12*Iy;
K2_lon = 20*Iy;

fprintf('Inner-loop gains used:\n');
fprintf('K1_lat = %.8f\n', K1_lat);
fprintf('K2_lat = %.8f\n', K2_lat);
fprintf('K1_lon = %.8f\n', K1_lon);
fprintf('K2_lon = %.8f\n\n', K2_lon);

%% K3 sweep ranges
K3_lat_vec = linspace(0, 0.02, 2000);
K3_lon_vec = linspace(0, 0.02, 2000);

eig_lat = zeros(3, length(K3_lat_vec));
eig_lon = zeros(3, length(K3_lon_vec));

valid_lat = [];
valid_lon = [];

%% Lateral sweep
for i = 1:length(K3_lat_vec)
    K3 = K3_lat_vec(i);

    
    A_lat = [ 0          0           g;
             -K3/Ix   -K1_lat/Ix  -K2_lat/Ix;
              0          1           0 ];

    lam = eig(A_lat);
    eig_lat(:,i) = lam;

    
    if max(abs(imag(lam))) < 1e-8 && all(real(lam) < 0) && all(abs(real(lam)) >= 0.8)
        valid_lat = [valid_lat; K3 real(lam(:)).'];
    end
end

%% Longitudinal sweep
for i = 1:length(K3_lon_vec)
    K3 = K3_lon_vec(i);

   
    A_lon = [ 0          0          -g;
              K3/Iy   -K1_lon/Iy  -K2_lon/Iy;
              0          1           0 ];

    lam = eig(A_lon);
    eig_lon(:,i) = lam;

    if max(abs(imag(lam))) < 1e-8 && all(real(lam) < 0) && all(abs(real(lam)) >= 0.8)
        valid_lon = [valid_lon; K3 real(lam(:)).'];
    end
end

%% Plot lateral eigenvalue locus
figure;
hold on; grid on;
plot(real(eig_lat(1,:)), imag(eig_lat(1,:)), 'b.', 'MarkerSize', 8);
plot(real(eig_lat(2,:)), imag(eig_lat(2,:)), 'r.', 'MarkerSize', 8);
plot(real(eig_lat(3,:)), imag(eig_lat(3,:)), 'k.', 'MarkerSize', 8);
xline(-0.8, 'm--', 'LineWidth', 1.2);
xlabel('Real(\lambda)');
ylabel('Imag(\lambda)');
title('Problem 3.5: Lateral Eigenvalue Locus vs K_3');
legend('\lambda_1', '\lambda_2', '\lambda_3', '\tau = 1.25 s boundary', 'Location', 'best');

%% Plot longitudinal eigenvalue locus
figure;
hold on; grid on;
plot(real(eig_lon(1,:)), imag(eig_lon(1,:)), 'b.', 'MarkerSize', 8);
plot(real(eig_lon(2,:)), imag(eig_lon(2,:)), 'r.', 'MarkerSize', 8);
plot(real(eig_lon(3,:)), imag(eig_lon(3,:)), 'k.', 'MarkerSize', 8);
xline(-0.8, 'm--', 'LineWidth', 1.2);
xlabel('Real(\lambda)');
ylabel('Imag(\lambda)');
title('Problem 3.5: Longitudinal Eigenvalue Locus vs K_3');
legend('\lambda_1', '\lambda_2', '\lambda_3', '\tau = 1.25 s boundary', 'Location', 'best');

%% Print valid K3 ranges
fprintf('---------------------------------------------\n');
fprintf('LATERAL valid K3 values:\n');
if isempty(valid_lat)
    fprintf('No valid lateral K3 found in this search range.\n');
else
    fprintf('Min valid K3_lat = %.8f\n', valid_lat(1,1));
    fprintf('Max valid K3_lat = %.8f\n', valid_lat(end,1));
end

fprintf('\nLONGITUDINAL valid K3 values:\n');
if isempty(valid_lon)
    fprintf('No valid longitudinal K3 found in this search range.\n');
else
    fprintf('Min valid K3_lon = %.8f\n', valid_lon(1,1));
    fprintf('Max valid K3_lon = %.8f\n', valid_lon(end,1));
end
fprintf('---------------------------------------------\n');


if ~isempty(valid_lat)
    K3_lat_choice = valid_lat(round(end/2),1);

    A_lat_choice = [ 0                0               g;
                    -K3_lat_choice/Ix -K1_lat/Ix    -K2_lat/Ix;
                     0                1               0 ];

    fprintf('\nChosen K3_lat = %.8f\n', K3_lat_choice);
    disp('Lateral eigenvalues for chosen K3_lat:');
    disp(eig(A_lat_choice));
else
    K3_lat_choice = NaN;
end

if ~isempty(valid_lon)
    K3_lon_choice = valid_lon(round(end/2),1);

    A_lon_choice = [ 0                0              -g;
                     K3_lon_choice/Iy -K1_lon/Iy    -K2_lon/Iy;
                     0                1               0 ];

    fprintf('\nChosen K3_lon = %.8f\n', K3_lon_choice);
    disp('Longitudinal eigenvalues for chosen K3_lon:');
    disp(eig(A_lon_choice));
else
    K3_lon_choice = NaN;
end

%% Display final chosen gains
fprintf('\nFinal selected gains for Problem 3.5:\n');
fprintf('K1_lat = %.8f, K2_lat = %.8f, K3_lat = %.8f\n', K1_lat, K2_lat, K3_lat_choice);
fprintf('K1_lon = %.8f, K2_lon = %.8f, K3_lon = %.8f\n', K1_lon, K2_lon, K3_lon_choice);
