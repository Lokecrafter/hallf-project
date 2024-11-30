clear all; clc; close all;

%Inputdata
L = 10;

%Radiuses
axle_main_radius = 0.05/2;
axle_secondary_radius = axle_main_radius * 0.8;
chain_radius = axle_main_radius + 0.02; %rd
wheel_radius = 0.4/2; %Dd
brake_disk_radius = wheel_radius/2; % rb

%Positions
bearing_pos = 1; %b1
brake_disk_pos = 1;



%Forces
bearing_force_N = 20 * [0; 1; 1] / sqrt(2); % Actual values later
wheel_drive_force_N = 20 * [0; 0; 1];
wheel_normal_force_N = 20 * [0; 1; 0];
braking_force_N = 20 * [0; 1; 1] / sqrt(2);
chain_force_N = 20 * [0; 0; 1];

%Act points
wheel_act_point_left = [0; 0; -wheel_radius];
bearing_act_point_left = [bearing_pos; 0; 0];
brake_act_point_left = [brake_disk_pos; brake_disk_radius/sqrt(2); -brake_disk_radius/sqrt(2)];
chain_act_point = [L/2; 0; chain_radius];
brake_act_point_right = [L-brake_disk_pos; brake_disk_radius/sqrt(2); -brake_disk_radius/sqrt(2)];
bearing_act_point_right = [L-bearing_pos; 0; 0];
wheel_act_point_right = [L; 0; -wheel_radius];

wheel_total_force_N = wheel_normal_force_N + wheel_drive_force_N;


force_matrix = [wheel_total_force_N, bearing_force_N, braking_force_N, chain_force_N, braking_force_N, bearing_force_N, wheel_total_force_N];
act_point_matrix = [wheel_act_point_left, bearing_act_point_left, brake_act_point_left, chain_act_point, brake_act_point_right, bearing_act_point_right, wheel_act_point_right];

disp(force_matrix);
disp(act_point_matrix);

plot3(act_point_matrix(1, :), act_point_matrix(2, :), act_point_matrix(3, :), '*');
for i = 1:length(force_matrix)
    act_point = act_point_matrix(:, i);
    force = force_matrix(:, i);

    %Normalize force
    force = force / norm(force);
    force = force * 0.1;

    hold on;
    quiver3(act_point(1), act_point(2), act_point(3), force(1), force(2), force(3), 'r','LineWidth',1);
end

%Draw cylinders
%Main cylinder
hold on;
[X,Y,Z] = cylinder(axle_main_radius);
surf(Z*(L - 2 * bearing_pos) + bearing_pos, X, Y);

%Secondary cylinders
hold on;
[X,Y,Z] = cylinder(axle_secondary_radius);
surf(Z*bearing_pos, X, Y);
hold on;
surf(Z*bearing_pos + L -bearing_pos, X, Y);



pbaspect([1,1,1]);



function ret = calc_cross_section_forces(x_points, force_matrix, act_point_matrix)
    T = [];
    M = [];
    for x = x_points
        cross_section_pos = [x; 0; 0];
        forces = [];

        [~, cols] = size(act_point_matrix);
        %Get forces left of x
        for i = 1:cols
            if act_point_matrix(1, i) - x <= 0
                forces = [forces, force_matrix(:, i)];
            end
        end

        [~, cols] = size(forces);
        moments = zeros(size(forces));
        for i = 1:cols
            moments(:,i) = cross(act_point_matrix(:,i) - cross_section_pos, forces(:, i));
        end

        T = [T, -[sum(forces(1,:)); sum(forces(2,:)); sum(forces(3,:))]];
        M = [M, -[sum(moments(1,:)); sum(moments(2,:)); sum(moments(3,:))]];
    end 
    ret.T = T;
    ret.M = M;
end

force_matrix = [
    0, 0, 0;
    0, 0, 0;
    1, 2, 1;
];
act_point_matrix = [
    0, 0.5, 1;
    0, 0, 0;
    0, 0, 0;
];

xx = linspace(0, 1, 100);

% result = calc_cross_section_forces(xx, force_matrix, act_point_matrix)
% disp(result.T)
% disp(result.M)
% f2 = figure;
% plot(xx, result.T(3,:), 'o-');
% hold on;
% plot(xx, result.M(2,:), 'o-');
% hold on;

result = calc_cross_section_forces(0, force_matrix, act_point_matrix);
disp(result.T)
disp(result.M)