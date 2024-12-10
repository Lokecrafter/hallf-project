clear all; clc; close all;

import Axle.*;
import Car.*;

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

b1 = 1; %Distance bearing
bb = 1; %Distance brake disk from center

D = 1; %Diameter main
d = 0.6 * D; %Diameter secondary



%Veichle data
air_resistance_coefficient = 1;
veichle_front_area = L*(h+h1);

%Constants
air_desity = 1;
mass = 100;
g = 9.82;


%Inputs
velocity = 10;

car = Car(df, db, h1, h, L, mass, air_resistance_coefficient, veichle_front_area);
axle = Axle(L, b1, bb, D/2, d/2, rb, rd, dh/2, car);

axle = axle.update_load_constant_velocity(velocity);



%Main program

disp(axle)

disp(axle.force_matrix);
disp(axle.act_point_matrix);

%Plot 3D visualization
plot3(axle.act_point_matrix(1, :), axle.act_point_matrix(2, :), axle.act_point_matrix(3, :), '*');
for i = 1:length(axle.force_matrix)
    act_point = axle.act_point_matrix(:, i);
    force = axle.force_matrix(:, i);
    
    %Normalize force
    force = force / norm(force);
    force = force * 0.1;
    
    hold on;
    quiver3(act_point(1), act_point(2), act_point(3), force(1), force(2), force(3), 'r','LineWidth',1);
end

%Draw cylinders
%Main cylinder
hold on;
[X,Y,Z] = cylinder(axle.radius_main);
surf(Z*(L - 2 * axle.distance_bearing) + axle.distance_bearing, X, Y);

%Secondary cylinders
hold on;
[X,Y,Z] = cylinder(axle.radius_secondary);
surf(Z*axle.distance_bearing, X, Y);
hold on;
surf(Z*axle.distance_bearing + L - axle.distance_bearing, X, Y);



pbaspect([1,1,1]);


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
        
        function plot_cross_section_forces_and_moments(axle)
            xx = linspace(0, axle.length_axle, 100);
            
            result = axle.calc_cross_section_forces(xx);
            disp(result.T)
            disp(result.M)

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
        end
        

        velocity = 10;
        acceleration = 1;
        deacceleration = -1;

        f1 = figure("Name", "Constant velocity");
        axle = axle.update_load_constant_velocity(velocity);
        plot_cross_section_forces_and_moments(axle);
        
        f2 = figure("Name", "Acceleration");
        axle = axle.update_load_acceleration(velocity, acceleration);
        plot_cross_section_forces_and_moments(axle);

        f3 = figure("Name", "Braking");
        axle = axle.update_load_acceleration(velocity, deacceleration);
        plot_cross_section_forces_and_moments(axle);


        % result = calc_cross_section_forces(0, force_matrix, act_point_matrix);
        % disp(result.T)
        % disp(result.M)
        
        % result = calc_cross_section_stress(0.3, result.T, result.M, cross_section_radiuses, cross_section_change_area_position);
        % disp("Stress matrixes: ")
        % disp(result);