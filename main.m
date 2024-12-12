clear all; clc; close all;

import Axle.*;
import Car.*;

%Inputdata
L = 1.100;

%Veichle geometry
h = 0.45;  %CoM - Ground
h1 = 0.2; %CoM - Air resistance
df = 0.800; %CoM - front
db = 0.300; %CoM - rear
dh = 0.520; %Wheel diameter
rb = 0.35 * dh; %radius brake rotor
rd = 0.4 * dh; %radius chain drive

b1 = 0.150; %Distance bearing
bb = 0.1; %Distance brake disk from center
bb = 0.3; %Distance brake disk from center

D = 0.050; %Diameter main
d = 0.6 * D; %Diameter secondary



%Veichle data
air_resistance_coefficient = 0.3;
veichle_front_area = 0.5;

%Constants
air_desity = 1.2;
mass = 150;
g = 9.82;


%Inputs
velocity = 95/3.6;

car = Car(df, db, h1, h, L, mass, air_resistance_coefficient, veichle_front_area);
axle = Axle(L, b1, bb, D/2, d/2, rb, rd, dh/2, car);

axle = axle.update_load_constant_velocity(velocity);



%Main program

disp(axle)

disp(axle.force_matrix);
disp(axle.act_point_matrix);




        
function plot_cross_section_forces_and_moments(axle)
    xx = linspace(0, axle.length_axle, 100);
    
    result = axle.calc_cross_section_forces(xx);
    max_stresses = axle.calc_max_cross_section_effective_max_stress(xx, result.T, result.M);

    %Plot 3D visualization
    subplot(2, 3, 4);
    plot3(axle.act_point_matrix(1, :), axle.act_point_matrix(2, :), axle.act_point_matrix(3, :), '*');
    [~,cols] = size(axle.force_matrix);
    for i = 1:cols
        act_point = axle.act_point_matrix(:, i);
        force = axle.force_matrix(:, i);
        
        %Normalize force
        force = force / norm(force);
        force = force * 0.1;
        
        hold on;
        quiver3(act_point(1), act_point(2), act_point(3), force(1), force(2), force(3), 'r','LineWidth',1);
    end

    %Draw direction arrow
    quiver3(axle.length_axle/2, 0, -axle.radius_wheel, 0, axle.radius_wheel * 2, 0, 'r', 'LineWidth', 5,'Color', "#bed466");

    %Draw cylinders
    %Main cylinder
    [X,Y,Z] = cylinder(axle.radius_main);
    hold on;    s1 = surf(Z*(axle.length_axle - 2 * axle.distance_bearing) + axle.distance_bearing, X, Y);

    %Secondary cylinders
    [X,Y,Z] = cylinder(axle.radius_secondary);
    hold on;    s2 = surf(Z*axle.distance_bearing, X, Y);
    hold on;    s3 = surf(Z*axle.distance_bearing + axle.length_axle - axle.distance_bearing, X, Y);

    s1.EdgeColor = "none";
    s2.EdgeColor = "none";
    s3.EdgeColor = "none";
    s1.FaceColor = "interp";
    s2.FaceColor = "interp";
    s3.FaceColor = "interp";
    s1.FaceLighting = "gouraud";
    s2.FaceLighting = "gouraud";
    s3.FaceLighting = "gouraud";
    axis equal
    camproj perspective



    subplot(2, 3, 1);
    plot(xx, result.T(1,:), 'o-');
    hold on;
    plot(xx, result.M(1,:), 'o-');
    hold on;
    title("X direction (right)");
    legend(["Tx", "Mx"])
    
    subplot(2, 3, 2);
    plot(xx, result.T(2,:), 'o-');
    hold on;
    plot(xx, result.M(2,:), 'o-');
    hold on;
    title("Y direction (forward)");
    legend(["Ty", "My"])
    
    subplot(2, 3, 3);
    plot(xx, result.T(3,:), 'o-');
    hold on;
    plot(xx, result.M(3,:), 'o-');
    hold on;
    title("Z direction (up)");
    legend(["Tz", "Mz"])

    subplot(2, 3, 5);
    plot(xx, sqrt(result.T(2,:).^2 + result.T(3,:).^2), 'o-');
    hold on;
    plot(xx, sqrt(result.M(2,:).^2 + result.M(3,:).^2), 'o-');
    hold on;
    plot(xx, result.M(1,:), 'o-');
    hold on;
    legend(["Tvärkraft belopp", "Böjmoment belopp", "Vridmoment belopp"])
    title("Magnitudes");
    % legend(["Tvärkraft belopp", "Böjmoment belopp"])
    
    subplot(2, 3, 6);
    plot(xx, max_stresses ./ 1e6, "o-");
    ylabel("Effective stress [MPa]");
    title("Effective stress");
    legend(["Effective stress"]);

end


velocity = 95/3.6;
acceleration = 6;
deacceleration = -15;

f1 = figure("Name", "Constant velocity");
axle = axle.update_load_constant_velocity(velocity);
plot_cross_section_forces_and_moments(axle);

f2 = figure("Name", "Acceleration");
axle = axle.update_load_acceleration(velocity, acceleration);
plot_cross_section_forces_and_moments(axle);

f3 = figure("Name", "Braking");
axle = axle.update_load_braking(velocity, deacceleration);
plot_cross_section_forces_and_moments(axle);



f4 = figure("Name", "Pure torsion");
radius = 1;
axle = Axle(1, 0.1, 0.1, radius, radius, 0, 0, 0, car);
force_matrix = [
    0,0, 0, 0;
    1, -1, 1, -1;
    0, 0, 0, 0
] * 1000;
act_point_matrix = [
    0, 0, 1, 1;
    0, 0, 0, 0;
    -1, 1, 1, -1
];
axle.force_matrix = force_matrix;
axle.act_point_matrix = act_point_matrix;
plot_cross_section_forces_and_moments(axle);
f4 = figure("Name", "Pure bending");
force_matrix = [
    0, 0, 0;
    0,0, 0,;
    1, -2, 1,
] * 1000;
act_point_matrix = [
    0, 0.5, 1;
    0, 0, 0;
    -1,-1,-1,
];
axle.force_matrix = force_matrix;
axle.act_point_matrix = act_point_matrix;
plot_cross_section_forces_and_moments(axle);
f4 = figure("Name", "Pure tension");
force_matrix = [
    -1, 1;
    0, 0,;
    0, 0,
] * 1000;
act_point_matrix = [
    0, 1;
    0, 0;
    0, 0,
];
axle.force_matrix = force_matrix;
axle.act_point_matrix = act_point_matrix;
plot_cross_section_forces_and_moments(axle);

% result = calc_cross_section_forces(0, force_matrix, act_point_matrix);
% disp(result.T)
% disp(result.M)

% result = calc_cross_section_stress(0.3, result.T, result.M, cross_section_radiuses, cross_section_change_area_position);
% disp("Stress matrixes: ")
% disp(result);