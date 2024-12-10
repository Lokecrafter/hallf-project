clear all; clc; close all;

%Inputdata
L = 10;

%Veichle geometry
h = 1;  %CoM - Ground
h1 = 0.5; %CoM - Air resistance
df = 1; %CoM - front
db = 1; %CoM - rear
dh = 2; %Wheel diameter
rd = 1; %radius chain drive
rb = 1; %radius brake rotor
Dd = 1; %radius wheel

%Constants
air_desity = 1;
veichle_front_area = L*(h+h1);
air_resistance_coefficient = 1;
a = 0; %acceleration
v = 10; %velocity
mass = 100;
g = 9.82;

%Given forces
air_resistance_magnitude = 0.5 * air_desity * veichle_front_area * air_resistance_coefficient * v * v;
drive_force_magnitude = air_resistance_magnitude + mass * a;
wheel_force_vertical_magnitude = (drive_force_magnitude * h + air_resistance_magnitude * h1 + mass * g * df) / (db + df);
chain_force_magnitude = drive_force_magnitude * dh / (2 * rd);

%Radiuses
axle_main_radius = rb / 2;
axle_secondary_radius = axle_main_radius * 0.8;
chain_radius = rd; %rd
wheel_radius = Dd; %Dd
brake_disk_radius = rb; % rb




%Cross section information
cross_section_radiuses = [axle_secondary_radius, axle_main_radius, axle_secondary_radius];
cross_section_change_area_position = [0, bearing_pos, L-bearing_pos];
% cross_section_moment_of_inertias = [
%     0, 0, 0;
%     axle_secondary_radius, axle_main_radius, axle_secondary_radius;
%     axle_secondary_radius, axle_main_radius, axle_secondary_radius
% ].^4 * pi / 4;

% cross_section_areas = [axle_secondary_radius, axle_main_radius, axle_secondary_radius].^2 * pi;










%Main program

disp(force_matrix);
disp(act_point_matrix);

%Plot 3D visualization
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



function ret = calc_cross_section_stress(x_points, cross_section_forces, cross_section_moments, cross_section_radiuses, cross_section_change_area_position)
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

% force_matrix = [
%     0, 0, 0;
%     0, 0, 0;
%     1, -2, 1;
% ];
% act_point_matrix = [
%     0, 0.5, 1;
%     0, 0, 0;
%     0, 0, 0;
% ];

xx = linspace(0, L, 100);

result = calc_cross_section_forces(xx, force_matrix, act_point_matrix);
disp(result.T)
disp(result.M)
f2 = figure;
subplot(2, 2, 1);
plot(xx, result.T(1,:), 'o-');
hold on;
plot(xx, result.M(1,:), 'o-');
hold on;
title("X direction (right)");
legend(["Tx", "Mx"])

subplot(2, 2, 2);
plot(xx, result.T(2,:), 'o-');
hold on;
plot(xx, result.M(2,:), 'o-');
hold on;
title("Y direction (forward)");
legend(["Ty", "My"])

subplot(2, 2, 3);
plot(xx, result.T(3,:), 'o-');
hold on;
plot(xx, result.M(3,:), 'o-');
hold on;
title("Z direction (up)");
legend(["Tz", "Mz"])

% result = calc_cross_section_forces(0, force_matrix, act_point_matrix);
% disp(result.T)
% disp(result.M)

result = calc_cross_section_stress(0.3, result.T, result.M, cross_section_radiuses, cross_section_change_area_position);
disp("Stress matrixes: ")
disp(result);