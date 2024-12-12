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
            air_resistance_magnitude = 0.5 * air_desity * obj.car.area_front * obj.car.coefficient_air_resistance * new_velocity.^2;

            %Given forces
            drive_force_total = air_resistance_magnitude;
            vertical_force_total = (air_resistance_magnitude * (obj.car.height_air_resistance+obj.car.height_center_of_mass) + obj.car.mass * g * obj.car.distance_front) / (obj.car.distance_rear + obj.car.distance_front);
            chain_force_total = drive_force_total * obj.radius_wheel/obj.raduis_drive;
            brake_force_total = 0;

            %Directions
            right = [1; 0; 0];
            forward = [0; 1; 0];
            up = [0; 0; 1];

            %Forces
            vect_wheel_force_total = forward * drive_force_total + up * vertical_force_total;
            vect_brake_force_total = brake_force_total * (up + forward) / norm(up + forward);
            vect_chain_force_total = chain_force_total * forward;

            %Reaction forces
            vect_beraing_force_total = -(vect_chain_force_total + vect_brake_force_total + vect_wheel_force_total);


            %Act points
            wheel_act_point_left = 0 * right - obj.radius_wheel * up;
            bearing_act_point_left = (obj.distance_bearing) * right;
            brake_act_point_left = (obj.length_axle * 0.5 - obj.distance_center2brake) * right - (obj.radius_brake/sqrt(2)) * forward + (obj.radius_brake/sqrt(2)) * up;
            chain_act_point = right * obj.length_axle/2 + obj.raduis_drive * up;
            brake_act_point_right = (obj.length_axle * 0.5 + obj.distance_center2brake) * right - (obj.radius_brake/sqrt(2)) * forward + (obj.radius_brake/sqrt(2)) * up;
            bearing_act_point_right = (obj.length_axle - obj.distance_bearing) * right;
            wheel_act_point_right = obj.length_axle * right - obj.radius_wheel * up;

            %Forces at acting points on the axle
            obj.force_matrix = [vect_wheel_force_total*0.5, vect_beraing_force_total*0.5, vect_brake_force_total*0.5, vect_chain_force_total, vect_brake_force_total*0.5, vect_beraing_force_total*0.5, vect_wheel_force_total*0.5];
            obj.act_point_matrix = [wheel_act_point_left, bearing_act_point_left, brake_act_point_left, chain_act_point, brake_act_point_right, bearing_act_point_right, wheel_act_point_right];

            ret = obj;
        end
        function ret=update_load_acceleration(obj, new_velocity, new_acceleration)
            %Constants
            air_desity = 1;
            g = 9.82;
            air_resistance_magnitude = 0.5 * air_desity * obj.car.area_front * obj.car.coefficient_air_resistance * new_velocity.^2;

            %Given forces
            drive_force_total = air_resistance_magnitude + obj.car.mass * new_acceleration;
            vertical_force_total = (air_resistance_magnitude * (obj.car.height_air_resistance+obj.car.height_center_of_mass) + obj.car.mass * new_acceleration * obj.car.height_center_of_mass + obj.car.mass * g * obj.car.distance_front) / (obj.car.distance_rear + obj.car.distance_front);
            chain_force_total = drive_force_total * obj.radius_wheel/obj.raduis_drive;
            brake_force_total = 0;

            %Directions
            right = [1; 0; 0];
            forward = [0; 1; 0];
            up = [0; 0; 1];

            %Forces
            vect_wheel_force_total = forward * drive_force_total + up * vertical_force_total;
            vect_brake_force_total = brake_force_total * (up + forward) / norm(up + forward);
            vect_chain_force_total = chain_force_total * forward;

            %Reaction forces
            vect_beraing_force_total = -(vect_chain_force_total + vect_brake_force_total + vect_wheel_force_total);


            %Act points
            wheel_act_point_left = 0 * right - obj.radius_wheel * up;
            bearing_act_point_left = (obj.distance_bearing) * right;
            brake_act_point_left = (obj.length_axle * 0.5 - obj.distance_center2brake) * right - (obj.radius_brake/sqrt(2)) * forward + (obj.radius_brake/sqrt(2)) * up;
            chain_act_point = right * obj.length_axle/2 + obj.raduis_drive * up;
            brake_act_point_right = (obj.length_axle * 0.5 + obj.distance_center2brake) * right - (obj.radius_brake/sqrt(2)) * forward + (obj.radius_brake/sqrt(2)) * up;
            bearing_act_point_right = (obj.length_axle - obj.distance_bearing) * right;
            wheel_act_point_right = obj.length_axle * right - obj.radius_wheel * up;

            %Forces at acting points on the axle
            obj.force_matrix = [vect_wheel_force_total*0.5, vect_beraing_force_total*0.5, vect_brake_force_total*0.5, vect_chain_force_total, vect_brake_force_total*0.5, vect_beraing_force_total*0.5, vect_wheel_force_total*0.5];
            obj.act_point_matrix = [wheel_act_point_left, bearing_act_point_left, brake_act_point_left, chain_act_point, brake_act_point_right, bearing_act_point_right, wheel_act_point_right];

            ret = obj;
        end
        function ret=update_load_braking(obj, new_velocity, new_acceleration)
            %Constants
            air_desity = 1;
            g = 9.82;
            air_resistance_magnitude = 0.5 * air_desity * obj.car.area_front * obj.car.coefficient_air_resistance * new_velocity.^2;

            %Given forces
            drive_force_total = air_resistance_magnitude + obj.car.mass * new_acceleration;
            vertical_force_total = (air_resistance_magnitude * (obj.car.height_air_resistance+obj.car.height_center_of_mass) + obj.car.mass * new_acceleration * obj.car.height_center_of_mass + obj.car.mass * g * obj.car.distance_front) / (obj.car.distance_rear + obj.car.distance_front);
            chain_force_total = 0;
            brake_force_total = drive_force_total * obj.radius_wheel/obj.radius_brake;

            %Directions
            right = [1; 0; 0];
            forward = [0; 1; 0];
            up = [0; 0; 1];

            %Forces
            vect_wheel_force_total = forward * drive_force_total + up * vertical_force_total;
            vect_brake_force_total = brake_force_total * (up + forward) / norm(up + forward);
            vect_chain_force_total = chain_force_total * forward;

            %Reaction forces
            vect_beraing_force_total = -(vect_chain_force_total + vect_brake_force_total + vect_wheel_force_total);


            %Act points
            wheel_act_point_left = 0 * right - obj.radius_wheel * up;
            bearing_act_point_left = (obj.distance_bearing) * right;
            brake_act_point_left = (obj.length_axle * 0.5 - obj.distance_center2brake) * right - (obj.radius_brake/sqrt(2)) * forward + (obj.radius_brake/sqrt(2)) * up;
            chain_act_point = right * obj.length_axle/2 + obj.raduis_drive * up;
            brake_act_point_right = (obj.length_axle * 0.5 + obj.distance_center2brake) * right - (obj.radius_brake/sqrt(2)) * forward + (obj.radius_brake/sqrt(2)) * up;
            bearing_act_point_right = (obj.length_axle - obj.distance_bearing) * right;
            wheel_act_point_right = obj.length_axle * right - obj.radius_wheel * up;

            %Forces at acting points on the axle
            obj.force_matrix = [vect_wheel_force_total*0.5, vect_beraing_force_total*0.5, vect_brake_force_total*0.5, vect_chain_force_total, vect_brake_force_total*0.5, vect_beraing_force_total*0.5, vect_wheel_force_total*0.5];
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
                    if obj.act_point_matrix(1, i) <= x
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

        function ret = calc_max_cross_section_effective_max_stress(obj, x_points, cross_section_forces, cross_section_moments)
            cross_section_radiuses = [obj.radius_secondary, obj.radius_main, obj.radius_secondary];
            cross_section_change_area_positions = [0, obj.distance_bearing, obj.length_axle - obj.distance_bearing];
            cross_section_moment_of_inertias = [
                0, 0, 0;
                cross_section_radiuses;
                cross_section_radiuses
            ].^4 * pi ./ 4;
            cross_section_areas = pi * cross_section_radiuses.^2;
                

            max_effecive_stresses = zeros(1, length(x_points));
            
            for i = 1:length(x_points)
                %Normal stress
                
                
                %Get cross section of x
                [~, cols] = size(cross_section_change_area_positions);
                cross_section_index = 1;
                for j = 1:cols
                    if cross_section_change_area_positions(j) <= x_points(i)
                        cross_section_index = j;

                    end
                end

                
                cs_force = cross_section_forces(:, i);
                cs_moment = cross_section_moments(:, i);
                cs_radius = cross_section_radiuses(cross_section_index);
                cs_area = cross_section_areas(cross_section_index);
                cs_inertia = cross_section_moment_of_inertias(:, cross_section_index);

                % stress_xx = [];
                % stress_xy = [];
                % stress_xz = [];

                % for angle_i = linspace(1, 360, 360)
                %     angle = 2 * pi * angle_i / 360;
                %     y_coordinate = cos(angle) * cs_radius;
                %     z_coordinate = sin(angle) * cs_radius;



                %     %           ---Normal stress x---            ---Bending stress y---                     ---Bending stress z---
                %     stress_xx = [stress_xx, cs_force(1) / cs_area + z_coordinate * cs_moment(2) / cs_inertia(2) + y_coordinate * cs_moment(3) / cs_inertia(3)];
                
                %     %Shear stress
                %     stress_xz = [stress_xz, 4*cs_force(3) / (3*pi * cs_radius.^4) * (cs_radius.^2 - z_coordinate.^2) + 4*cs_force(2) / (3*pi * cs_radius.^4) * z_coordinate*y_coordinate];
                %     stress_xy = [stress_xy, 4*cs_force(2) / (3*pi * cs_radius.^4) * (cs_radius.^2 - y_coordinate.^2) - 4*cs_force(3) / (3*pi * cs_radius.^4) * y_coordinate*z_coordinate];
                %     stress_xy = 0;  %Don't bother with shear stresses
                %     stress_xz = 0;  %DOn't bother with shear stresses
                % end
                
                % max_xx = max(stress_xx);
                % max_xy = max(stress_xy);
                % max_xz = max(stress_xz);

                %Found in project description
                max_xx = abs(cs_force(1) / cs_area) + abs(cs_radius * sqrt((cs_moment(3)*cs_inertia(2)).^2 + (cs_moment(2)*cs_inertia(3)).^2) / (cs_inertia(2)*cs_inertia(3)));
                max_xy = 0;
                max_xz = 0;

                effective_stress_bend = sqrt(max_xx.^2 + 3*max_xy.^2 + 3*max_xz.^2);

                %Torsion
                shear_torsion = cs_radius* cs_moment(1) / (cs_radius.^4 * pi / 2);
                effective_stress_torsion = sqrt(3) * abs(shear_torsion);

                max_effecive_stresses(i) = effective_stress_bend + effective_stress_torsion;
            end
        
            ret = max_effecive_stresses;
        end
        

    end
end
