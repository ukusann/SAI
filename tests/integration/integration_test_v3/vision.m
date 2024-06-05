classdef vision < handle
    %o rui é gay
    properties (Access = private)
        vrep
        clientID

        %Definição dos objetos das juntas

    end
    properties
        Visao
        cube_detected_color
        box_detected_color
        flag_detect_object
        number_of_detected_objects

    end
    methods
        function [obj,error] = vision(sim_obj)

            error = 0;

            [obj.vrep, obj.clientID] = sim_obj.get_connection();

            if obj.clientID <= -1
                clear obj;
                msg = 'ERROR: sim_obj seems to have an invalid connection to simulator!\n';
                error(msg);
            end

            %- parte mais recente
            [res, camhandle] = obj.vrep.simxGetObjectHandle(obj.clientID, 'Vision_sensor', obj.vrep.simx_opmode_oneshot_wait);


            [res, resolution, image1] = obj.vrep.simxGetVisionSensorImage2(obj.clientID, camhandle, 0, obj.vrep.simx_opmode_oneshot_wait);

            img=uint8(image1);
            imshow(image1);

        end
    end
end

