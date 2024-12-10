classdef Car
    properties
        distance_front
        distance_rear
        height_air_resistance
        height_center_of_mass
        width

        mass
        coefficient_air_resistance
        area_front
    end
    methods
        function obj=Car(distance_front, distance_rear, height_air_resistance, height_center_of_mass, width, mass, coefficient_air_resistance, area_front)
            import Solvers.*;
            if nargin == 8
                obj.distance_front = distance_front;
                obj.distance_rear = distance_rear;
                obj.height_air_resistance = height_air_resistance;
                obj.height_center_of_mass = height_center_of_mass;
                obj.width = width;
                obj.mass = mass;
                obj.coefficient_air_resistance = coefficient_air_resistance;
                obj.area_front = area_front;
            end
        end
    end
end