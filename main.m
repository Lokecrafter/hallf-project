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

%Cross section information
cross_section_radiuses = [axle_secondary_radius, axle_main_radius, axle_secondary_radius];
cross_section_locations = [0, bearing_pos, L-bearing_pos];
% cross_section_moment_of_inertias = [
%     0, 0, 0;
%     axle_secondary_radius, axle_main_radius, axle_secondary_radius;
%     axle_secondary_radius, axle_main_radius, axle_secondary_radius
% ].^4 * pi / 4;

% cross_section_areas = [axle_secondary_radius, axle_main_radius, axle_secondary_radius].^2 * pi;










%Main program

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

function ret = calc_cross_section_stress(x_points, cross_section_forces, cross_section_moments, cross_section_radiuses, cross_section_locations)
    stress_matrixes = zeros(3, 3, length(x_points));
    cross_section_moment_of_inertias = [
        0, 0, 0;
        cross_section_radiuses;
        cross_section_radiuses
    ].^4 * pi ./ 4;
    cross_section_areas = pi * cross_section_radiuses.^2;
    
    for i = 1:length(x_points)
        %Normal stress
        z_coordinate = 0.1;
        y_coordinate = 0.1;
        
        cs_force = cross_section_forces(:, i);
        cs_moment = cross_section_moments(:, i);
        cs_radius = cross_section_radiuses(1);
        cs_area = cross_section_areas(1);
        cs_inertia = cross_section_moment_of_inertias(:, 1);

        stress_xx = cs_force(1) / cs_area + y_coordinate * cs_moment(2) / cs_inertia(2) + z_coordinate * cs_moment(3) / cs_inertia(3);

        %Shear stress
        % stress_xz = 4/3 * cs_force / cs_area;
        % stress_xy = 4/3 * cs_force / cs_area;
        %Stress xz = force-z direction - force y-direction
        stress_xz = 4*cs_force(3) / (3*pi * cs_radius.^4) * (cs_radius.^2 - z_coordinate.^2) + 4*cs_force(2) / (3*pi * cs_radius.^4) * z_coordinate*y_coordinate;
        stress_xy = 4*cs_force(2) / (3*pi * cs_radius.^4) * (cs_radius.^2 - y_coordinate.^2) - 4*cs_force(3) / (3*pi * cs_radius.^4) * y_coordinate*z_coordinate;
        % stress_xy = -4 * cs_force(q) / (3 * pi * cs_radius.^4) * y_coordinate * z_coordinate;

        % stress_twist = G * Mv / Kr * r
        stress_twist_magnitude = sqrt(y_coordinate.^2 * z_coordinate.^2) * cs_moment(1) / (cs_radius.^4 * pi / 2);
        stress_twist = [0; -z_coordinate; y_coordinate];
        stress_twist = stress_twist_magnitude * stress_twist / norm(stress_twist);
        disp("Stress twist:   ");
        disp(stress_twist);

        stress_matrix = [
            stress_xx, stress_xy, stress_xz;
            stress_xy, 0, 0;
            stress_xz, 0, 0
        ];
        stress_matrixes(:,:,i) = stress_matrix;
        end

    ret = stress_matrixes;
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

result = calc_cross_section_forces(xx, force_matrix, act_point_matrix)
disp(result.T)
disp(result.M)
f2 = figure;
plot(xx, result.T(3,:), 'o-');
hold on;
plot(xx, result.M(2,:), 'o-');
hold on;

% result = calc_cross_section_forces(0, force_matrix, act_point_matrix);
% disp(result.T)
% disp(result.M)

result = calc_cross_section_stress(0.3, result.T, result.M, cross_section_radiuses, cross_section_locations);
disp("Stress matrixes: ")
disp(result);