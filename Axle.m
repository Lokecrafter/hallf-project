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

            %Positions
            bearing_pos = 1; %b1
            brake_disk_pos = 1.5;

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
            wheel_act_point_left = 0 * right - wheel_radius * up;
            bearing_act_point_left = bearing_pos * right;
            brake_act_point_left = brake_disk_pos * right + (brake_disk_radius/sqrt(2)) * forward - (brake_disk_radius/sqrt(2)) * up;
            chain_act_point = right * L/2 + chain_radius * up;
            brake_act_point_right = (L-brake_disk_pos) * right + (brake_disk_radius/sqrt(2)) * forward - (brake_disk_radius/sqrt(2)) * up;
            bearing_act_point_right = (L-bearing_pos) * right;
            wheel_act_point_right = L * right - wheel_radius * up;

            wheel_total_force_N = wheel_normal_force_N + wheel_drive_force_N;

            %Forces at acting points on the axle
            force_matrix = [wheel_total_force_N, bearing_force_N, braking_force_N, chain_force_N, braking_force_N, bearing_force_N, wheel_total_force_N];
            act_point_matrix = [wheel_act_point_left, bearing_act_point_left, brake_act_point_left, chain_act_point, brake_act_point_right, bearing_act_point_right, wheel_act_point_right];

            
        end
    end
end
