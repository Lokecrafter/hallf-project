classdef Axle
    properties
        length {mustBeNumeric}
        distance_bearing {mustBeNumeric}
        distance_center2brake {mustBeNumeric}


        radius_main {mustBeNumeric}
        radius_secondary {mustBeNumeric}

        radius_brake {mustBeNumeric}
        raduis_drive {mustBeNumeric}
        radius_wheel {mustBeNumeric}

        car

        force_matrix
        act_point_matrix
    end
    methods
        function obj=Axle(length, distance_bearing, distance_center2brake, radius_main, radius_secondary, radius_brake, raduis_drive, radius_wheel, car)
            import Solvers.*;
            if nargin == 9
                obj.length;
                obj.distance_bearing;
                obj.distance_center2brake;
                obj.radius_main;
                obj.radius_secondary;
                obj.radius_brake;
                obj.raduis_drive;
                obj.radius_wheel;
                obj.car;
            end
        end
        function update_load_constant_velocity(obj, new_velocity)
            %Inputdata
            L = 10;

            %Veichle geometry
            h = 1;  %CoM - Ground
            h1 = 0.5; %CoM - Air resistance
            df = 1; %CoM - front
            db = 1; %CoM - rear
            dh = 2; %
            rd = 1; %radius chain drive
            rb = 1; %radius brake rotor
            Dd = 1; %radius wheel

            %Constants
            air_desity = 1;
            g = 9.82;

            car = obj.car;
            %Given forces
            air_resistance_magnitude = 0.5 * air_desity * car.area_front * car.coefficient_air_resistance * new_velocity.^2;
            drive_force_magnitude = air_resistance_magnitude + car.mass * 0;
            wheel_force_vertical_magnitude = (drive_force_magnitude * car.height_center_of_mass + air_resistance_magnitude * car.height_air_resistance + car.mass * g * car.distance_front) / (car.distance_rear + car.distance_front);
            chain_force_magnitude = drive_force_magnitude * (car.radius_wheel * 2) / (2 * obj.raduis_drive);

            %Directions
            right = [1; 0; 0];
            forward = [0; 1; 0];
            up = [0; 0; 1];

            %Forces
            bearing_force_N = -0.5 * (drive_force_magnitude + chain_force_magnitude) * forward + wheel_force_vertical_magnitude * -0.5 * up ; % Actual values later
            wheel_drive_force_N = drive_force_magnitude * 0.5 * forward;
            wheel_normal_force_N = wheel_force_vertical_magnitude * 0.5 * up;
            braking_force_N = 0 * (up + forward) / norm(up + forward);
            chain_force_N = chain_force_magnitude * forward;

            %Act points
            wheel_act_point_left = 0 * right - obj.radius_wheel * up;
            bearing_act_point_left = (obj.distance_bearing) * right;
            brake_act_point_left = (obj.length * 0.5 - obj.distance_center2brake) * right + (obj.radius_brake/sqrt(2)) * forward - (obj.radius_brake/sqrt(2)) * up;
            chain_act_point = right * L/2 + obj.raduis_drive * up;
            brake_act_point_right = (obj.length * 0.5 + obj.distance_center2brake) * right + (obj.radius_brake/sqrt(2)) * forward - (obj.radius_brake/sqrt(2)) * up;
            bearing_act_point_right = (obj.length - obj.distance_bearing) * right;
            wheel_act_point_right = obj.length * right - obj.radius_wheel * up;

            wheel_total_force_N = wheel_normal_force_N + wheel_drive_force_N;

            %Forces at acting points on the axle
            obj.force_matrix = [wheel_total_force_N, bearing_force_N, braking_force_N, chain_force_N, braking_force_N, bearing_force_N, wheel_total_force_N];
            obj.act_point_matrix = [wheel_act_point_left, bearing_act_point_left, brake_act_point_left, chain_act_point, brake_act_point_right, bearing_act_point_right, wheel_act_point_right];
        end






        %Calculate cross section force at x_points along x-axis
        function ret = calc_cross_section_forces(obj, x_points)
            T = [];
            M = [];
            for x = x_points
                cross_section_pos = [x; 0; 0];
                forces = [];
                
                [~, cols] = size(obj.act_point_matrix);
                %Get forces left of x
                for i = 1:cols
                    if obj.act_point_matrix(1, i) - x <= 0
                        forces = [forces, obj.force_matrix(:, i)];
                    end
                end

                [~, cols] = size(forces);
                moments = zeros(size(forces));
                for i = 1:cols
                    moments(:,i) = cross(obj.act_point_matrix(:,i) - cross_section_pos, forces(:, i));
                end
                
                T = [T, -[sum(forces(1,:)); sum(forces(2,:)); sum(forces(3,:))]];
                M = [M, -[sum(moments(1,:)); sum(moments(2,:)); sum(moments(3,:))]];
            end 
            ret.T = T;
            ret.M = M;
        end

    end
end
