classdef visao < handle
    properties (Access = public)
        vrep
        clientID
        Visao
        cir_red_x
        cir_red_y
        cube_red_x
        cube_red_y
        cube_green_x
        cube_green_y
        cir_green_x
        cir_green_y
        cube_blue_x
        cube_blue_y
        cir_blue_x
        cir_blue_y
        red_bowl
        green_bowl
        blue_bowl
        hue_red
        adjusted_green_channel
        hue_blue
        red_mask
        green_mask
        blue_mask
        final_mask
        final_img
        pixelCounts_new
        z_coordinate
    end

    methods
        function [obj,error] = visao(sim_obj)
            error = 0;
            [obj.vrep, obj.clientID] = sim_obj.get_connection();

            if obj.clientID <= -1
                clear obj;
                msg = 'ERROR: sim_obj seems to have an invalid connection to simulator!\n';
                error(msg);
            end
            %------------------------------------------Visão por computador---------

            [res, camhandle] = obj.vrep.simxGetObjectHandle(obj.clientID, 'Vision_sensor', obj.vrep.simx_opmode_oneshot);

            [res, resolution, image1] = obj.vrep.simxGetVisionSensorImage2(obj.clientID, camhandle, 0, obj.vrep.simx_opmode_oneshot);

            image=uint8(image1);
            imshow(image);

        %     %------------------------------------Visão-----------------------------------
        %     obj.red_bowl = 100*[0.415;1.775;0.3094];
        %     obj.green_bowl = 100*[0.790;1.0025; 0.3094];
        %     obj.blue_bowl = 100*[0.565; 1.0025; 0.3094];
        %     obj.z_coordinate = 0.3094+0.15;
        % 
        %     %-----------------------------------
        %     % Separation of red, green, and blue channels
        %     red_channel = image(:, :, 1);
        %     green_channel = image(:, :, 2);
        %     blue_channel = image(:, :, 3);
        % 
        %     % Define color range of interest
        %     lower_threshold = 245;
        %     upper_threshold = 255;
        % 
        %     % Create masks for red, green, and blue channels
        %     red_mask = (red_channel >= lower_threshold) & (red_channel <= upper_threshold);
        %     green_mask = (green_channel >= lower_threshold) & (green_channel <= upper_threshold);
        %     blue_mask = (blue_channel >= lower_threshold) & (blue_channel <= upper_threshold);
        % 
        %     final_mask = red_mask + green_mask + blue_mask;
        % 
        %     % Apply masks to original image
        %     red_objects = image .* uint8(red_mask);
        %     green_objects = image .* uint8(green_mask);
        %     blue_objects = image .* uint8(blue_mask);
        % 
        %     % Detection and plotting for red objects
        %     red_objects_gray = rgb2gray(red_objects);
        %     binary_red_objects = imbinarize(red_objects_gray, graythresh(red_objects_gray));
        %     se = strel('square', 8);
        %     processed_red_objects = imdilate(imerode(binary_red_objects, se), se);
        %     processed_red_objects = imfill(processed_red_objects, 'holes');
        % 
        %     % figure;
        %     % imshow(image);
        %     % title('Objects Detection');
        % 
        %     % hold on;
        % 
        %     props_red = regionprops(processed_red_objects, 'Centroid');
        % 
        %     for k = 1:length(props_red)
        %         centroid = props_red(k).Centroid;
        % 
        %         if (k == 1)
        %             obj.cube_red_x = (-0.001*centroid(1)) + 0.7725;
        %             obj.cube_red_y = (0.001*centroid(1)) + 1.2028;
        %             disp('Cube vermelho:');
        %             disp(obj.cube_red_x);
        %             disp(obj.cube_red_y);
        %             disp(obj.z_coordinate);
        %         elseif (k == 2)
        %             obj.cir_red_x = (-0.001*centroid(1)) + 0.7725;
        %             obj.cir_red_y = (0.001*centroid(1)) + 1.2028;
        %             disp('Cilindro vermelho:');
        %             disp(obj.cir_red_x);
        %             disp(obj.cir_red_y);
        %             disp(obj.z_coordinate);
        %         end
        % 
        %         % plot(centroid(1), centroid(2), 'ro', 'MarkerFaceColor', 'black', 'MarkerSize', 5);
        %         legend_text = sprintf('(%d, %d)', round(centroid(1)), round(centroid(2)));
        %         text(centroid(1) + 12, centroid(2) + 12, legend_text, 'Color', 'r', 'FontSize', 10);
        %     end
        % 
        %     % Detection and plotting for green objects
        %     green_objects_gray = rgb2gray(green_objects);
        %     binary_green_objects = imbinarize(green_objects_gray, graythresh(green_objects_gray));
        %     processed_green_objects = imdilate(imerode(binary_green_objects, se), se);
        %     processed_green_objects = imfill(processed_green_objects, 'holes');
        % 
        %     props_green = regionprops(processed_green_objects, 'Centroid');
        % 
        %     for k = 1:length(props_green)
        %         centroid = props_green(k).Centroid;
        % 
        %         if (k == 1)
        %             obj.cir_green_x = (-0.001*centroid(1)) + 0.7725;
        %             obj.cir_green_y = (0.001*centroid(1)) + 1.2028;
        %             disp('Cilindro verde:');
        %             disp(obj.cir_green_x);
        %             disp(obj.cir_green_y);
        %             disp(obj.z_coordinate);
        %         elseif (k == 2)
        %             obj.cube_green_x = (-0.001*centroid(1)) + 0.7725;
        %             obj.cube_green_y = (0.001*centroid(1)) + 1.2028;
        %             disp('Cube verde:');
        %             disp(obj.cube_green_x);
        %             disp(obj.cube_green_y);
        %             disp(obj.z_coordinate);
        %         end
        % 
        %         % plot(centroid(1), centroid(2), 'go', 'MarkerFaceColor', 'black', 'MarkerSize', 5);
        %         legend_text = sprintf('(%d, %d)', round(centroid(1)), round(centroid(2)));
        %         text(centroid(1) + 12, centroid(2) + 12, legend_text, 'Color', 'g', 'FontSize', 10);
        %     end
        % 
        %     blue_objects_gray = rgb2gray(blue_objects);
        %     binary_blue_objects = imbinarize(blue_objects_gray, graythresh(blue_objects_gray));
        %     processed_blue_objects = imdilate(imerode(binary_blue_objects, se), se);
        %     processed_blue_objects = imfill(processed_blue_objects, 'holes');
        % 
        %     props_blue = regionprops(processed_blue_objects, 'Centroid');
        % 
        %     for k = 1:length(props_blue)
        %         centroid = props_blue(k).Centroid;
        % 
        %         if (k == 1)
        %             obj.cube_blue_x = (-0.001*centroid(1)) + 0.7725;
        %             obj.cube_blue_y = (0.001*centroid(1)) + 1.2028;
        %             disp('Cubo azul:');
        %             disp(obj.cube_blue_x);
        %             disp(obj.cube_blue_y);
        %             disp(obj.z_coordinate);
        %         elseif (k == 2)
        %             obj.cir_blue_x = (-0.001*centroid(1)) + 0.7725;
        %             obj.cir_blue_y = (0.001*centroid(1)) + 1.2028;
        %             disp('Cilindro azul:');
        %             disp(obj.cir_blue_x);
        %             disp(obj.cir_blue_y);
        %             disp(obj.z_coordinate);
        %         end
        % 
        %         % plot(centroid(1), centroid(2), 'bo', 'MarkerFaceColor', 'black', 'MarkerSize', 5);
        %         legend_text = sprintf('(%d, %d)', round(centroid(1)), round(centroid(2)));
        %         text(centroid(1) + 12, centroid(2) + 12, legend_text, 'Color', 'b', 'FontSize', 10);
        %     end
        %     disp('Coordenadas Bowl vermelho:');
        %     disp (obj.red_bowl);
        %     disp('Coordenadas Bowl verde:');
        %     disp (obj.green_bowl);
        %     disp('Coordenadas Bowl azul:');
        %     disp (obj.blue_bowl);
        % 
        %     final_img = image.*uint8(final_mask);
        %     img_final_hsv = rgb2hsv(final_img);
        %     hue = img_final_hsv(:,:,1);
        %     hue = hue(find(final_mask));
        %     % figure;
        %     [pixelCounts, HUELevels]= imhist(hue);
        %     HUELevels = HUELevels.*180;
        %     % plot(HUELevels, pixelCounts);
        %     % axis([0 180 0 3000])
        %     % figure
        % 
        %     new_scale = 0:1:180;
        %     obj.pixelCounts_new = zeros(size(new_scale));
        % 
        %     for i = 1:length(new_scale)
        %         values = 0;
        %         sum = 0;
        %         for i2 = 1:length(HUELevels)
        %             if(uint8(HUELevels(i2)) == new_scale(i))
        %                 values = values + 1;
        %                 sum = pixelCounts(i2) + sum;
        %             end
        %         end
        %         if(values > 0)
        %             obj.pixelCounts_new(i) = sum/values;
        %         else
        %             obj.pixelCounts_new(i) = 0;
        %         end
        % 
        %     end
        %     % plot(new_scale,obj.pixelCounts_new);
        %     % axis([0 180 0 3000]);
        % 
         end
    end
end