classdef Axle
    properties
        length_axle {mustBeNumeric}
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

        cross_section_radiuses
        cross_section_change_area_positions
    end
    methods
        function obj=Axle(length_axle, distance_bearing, distance_center2brake, radius_main, radius_secondary, radius_brake, raduis_drive, radius_wheel, car)
            import Car.*;
            if nargin == 9
                obj.length_axle = length_axle;
                obj.distance_bearing = distance_bearing;
                obj.distance_center2brake = distance_center2brake;
                obj.radius_main = radius_main;
                obj.radius_secondary = radius_secondary;
                obj.radius_brake = radius_brake;
                obj.raduis_drive = raduis_drive;
                obj.radius_wheel = radius_wheel;
                obj.car = car;

                cross_section_radiuses = [obj.radius_secondary, obj.radius_main, obj.radius_secondary];
                cross_section_change_area_positions = [0, obj.distance_bearing, obj.length_axle - obj.distance_bearing];
                %Cross section information
                % cross_section_moment_of_inertias = [
                %     0, 0, 0;
                %     axle_secondary_radius, axle_main_radius, axle_secondary_radius;
                %     axle_secondary_radius, axle_main_radius, axle_secondary_radius
                % ].^4 * pi / 4;

                % cross_section_areas = [axle_secondary_radius, axle_main_radius, axle_secondary_radius].^2 * pi;
            end
        end
        function ret=update_load_constant_velocity(obj, new_velocity)
            %Constants
            air_desity = 1;
            g = 9.82;

            %Given forces
            air_resistance_magnitude = 0.5 * air_desity * obj.car.area_front * obj.car.coefficient_air_resistance * new_velocity.^2;
            drive_force_magnitude = air_resistance_magnitude + obj.car.mass * 0;
            wheel_force_vertical_magnitude = (drive_force_magnitude * obj.car.height_center_of_mass + air_resistance_magnitude * obj.car.height_air_resistance + obj.car.mass * g * obj.car.distance_front) / (obj.car.distance_rear + obj.car.distance_front);
            chain_force_magnitude = drive_force_magnitude * (obj.radius_wheel * 2) / (2 * obj.raduis_drive);

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
            brake_act_point_left = (obj.length_axle * 0.5 - obj.distance_center2brake) * right + (obj.radius_brake/sqrt(2)) * forward - (obj.radius_brake/sqrt(2)) * up;
            chain_act_point = right * obj.length_axle/2 + obj.raduis_drive * up;
            brake_act_point_right = (obj.length_axle * 0.5 + obj.distance_center2brake) * right + (obj.radius_brake/sqrt(2)) * forward - (obj.radius_brake/sqrt(2)) * up;
            bearing_act_point_right = (obj.length_axle - obj.distance_bearing) * right;
            wheel_act_point_right = obj.length_axle * right - obj.radius_wheel * up;

            wheel_total_force_N = wheel_normal_force_N + wheel_drive_force_N;

            %Forces at acting points on the axle
            obj.force_matrix = [wheel_total_force_N, bearing_force_N, braking_force_N, chain_force_N, braking_force_N, bearing_force_N, wheel_total_force_N];
            obj.act_point_matrix = [wheel_act_point_left, bearing_act_point_left, brake_act_point_left, chain_act_point, brake_act_point_right, bearing_act_point_right, wheel_act_point_right];

            ret = obj;
        end
        function ret=update_load_acceleration(obj, new_velocity, new_acceleration)
            %Constants
            air_desity = 1;
            g = 9.82;

            %Given forces
            air_resistance_magnitude = 0.5 * air_desity * obj.car.area_front * obj.car.coefficient_air_resistance * new_velocity.^2;
            drive_force_magnitude = air_resistance_magnitude + obj.car.mass * new_acceleration;
            wheel_force_vertical_magnitude = (drive_force_magnitude * obj.car.height_center_of_mass + air_resistance_magnitude * obj.car.height_air_resistance + obj.car.mass * g * obj.car.distance_front) / (obj.car.distance_rear + obj.car.distance_front);
            chain_force_magnitude = drive_force_magnitude * (obj.radius_wheel * 2) / (2 * obj.raduis_drive);

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
            brake_act_point_left = (obj.length_axle * 0.5 - obj.distance_center2brake) * right + (obj.radius_brake/sqrt(2)) * forward - (obj.radius_brake/sqrt(2)) * up;
            chain_act_point = right * obj.length_axle/2 + obj.raduis_drive * up;
            brake_act_point_right = (obj.length_axle * 0.5 + obj.distance_center2brake) * right + (obj.radius_brake/sqrt(2)) * forward - (obj.radius_brake/sqrt(2)) * up;
            bearing_act_point_right = (obj.length_axle - obj.distance_bearing) * right;
            wheel_act_point_right = obj.length_axle * right - obj.radius_wheel * up;

            wheel_total_force_N = wheel_normal_force_N + wheel_drive_force_N;

            %Forces at acting points on the axle
            obj.force_matrix = [wheel_total_force_N, bearing_force_N, braking_force_N, chain_force_N, braking_force_N, bearing_force_N, wheel_total_force_N];
            obj.act_point_matrix = [wheel_act_point_left, bearing_act_point_left, brake_act_point_left, chain_act_point, brake_act_point_right, bearing_act_point_right, wheel_act_point_right];

            ret = obj;
        end
        function ret=update_load_braking(obj, new_velocity, new_acceleration)
            %Constants
            air_desity = 1;
            g = 9.82;

            %Given forces
            air_resistance_magnitude = 0.5 * air_desity * obj.car.area_front * obj.car.coefficient_air_resistance * new_velocity.^2;
            drive_force_magnitude = air_resistance_magnitude + obj.car.mass * new_acceleration;
            wheel_force_vertical_magnitude = (drive_force_magnitude * obj.car.height_center_of_mass + air_resistance_magnitude * obj.car.height_air_resistance + obj.car.mass * g * obj.car.distance_front) / (obj.car.distance_rear + obj.car.distance_front);
            chain_force_magnitude = drive_force_magnitude * (obj.radius_wheel * 2) / (2 * obj.raduis_drive);
            chain_force_magnitude = drive_force_magnitude * (obj.radius_wheel * 2) / (2 * obj.radius_brake);

            %Directions
            right = [1; 0; 0];
            forward = [0; 1; 0];
            up = [0; 0; 1];

            %Forces
            bearing_force_N = -0.5 * (drive_force_magnitude + chain_force_magnitude) * forward + wheel_force_vertical_magnitude * -0.5 * up ; % Actual values later
            wheel_drive_force_N = drive_force_magnitude * 0.5 * forward;
            wheel_normal_force_N = wheel_force_vertical_magnitude * 0.5 * up;
            braking_force_N = drive_force_magnitude * (up + forward) / norm(up + forward);
            chain_force_N = chain_force_magnitude * forward;

            %Act points
            wheel_act_point_left = 0 * right - obj.radius_wheel * up;
            bearing_act_point_left = (obj.distance_bearing) * right;
            brake_act_point_left = (obj.length_axle * 0.5 - obj.distance_center2brake) * right + (obj.radius_brake/sqrt(2)) * forward - (obj.radius_brake/sqrt(2)) * up;
            chain_act_point = right * obj.length_axle/2 + obj.raduis_drive * up;
            brake_act_point_right = (obj.length_axle * 0.5 + obj.distance_center2brake) * right + (obj.radius_brake/sqrt(2)) * forward - (obj.radius_brake/sqrt(2)) * up;
            bearing_act_point_right = (obj.length_axle - obj.distance_bearing) * right;
            wheel_act_point_right = obj.length_axle * right - obj.radius_wheel * up;

            wheel_total_force_N = wheel_normal_force_N + wheel_drive_force_N;

            %Forces at acting points on the axle
            obj.force_matrix = [wheel_total_force_N, bearing_force_N, braking_force_N, chain_force_N, braking_force_N, bearing_force_N, wheel_total_force_N];
            obj.act_point_matrix = [wheel_act_point_left, bearing_act_point_left, brake_act_point_left, chain_act_point, brake_act_point_right, bearing_act_point_right, wheel_act_point_right];

            ret = obj;
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

        function ret = calc_cross_section_stress(obj, x_points, cross_section_forces, cross_section_moments)
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
        

    end
end
