classdef vision < handle
    properties (Access = private)
        vrep
        clientID


    end
    properties
        camhandle
    end
    methods
        function [obj,error] = vision(sim_obj)

            error = 0;

            [obj.vrep, obj.clientID] = sim_obj.get_connection();

            if obj.clientID <= -1
                clear obj;
                disp('ERROR: sim_obj seems to have an invalid connection to simulator!\n');
                error = 1;
                return;
            end

            [res, obj.camhandle] = obj.vrep.simxGetObjectHandle(obj.clientID, 'Vision_sensor', obj.vrep.simx_opmode_oneshot_wait);
            if (res ~= obj.vrep.simx_return_ok)
                disp('ERROR: Failed getting object handle');
                error = 1;
                return;
            end
            
        end

        function [error, resolution, frame] = getFrame(obj)
            error = 0;
            [res, resolution, frame] = obj.vrep.simxGetVisionSensorImage2(obj.clientID, obj.camhandle, 0, obj.vrep.simx_opmode_oneshot_wait);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                disp('ERROR: Failed getting vision sensor information');
                error = 1;
                return;
            end
        end

        function [boxToPick] = processFrame(obj, frame)
            boxToPick = 0;
            try
                % Crop the frame to the left half
                [~, cols, ~] = size(frame);
                
                %Get contours from frame
                [contours_raw, frameSize] = obj.getContours(frame);

                if ~isempty(contours_raw) && contours_raw.NumObjects >= 1
                    % Extract the first object's contour
                    [~, cols_contour] = ind2sub(frameSize, contours_raw.PixelIdxList{1});
                    rightmost_col = max(cols_contour);

                    final_col = min(rightmost_col + 10, cols); % Ensure it doesn't go out of bounds
                    filteredFrame = frame(:, 1:final_col, :);

                    % Display the filtered image
                    imshow(filteredFrame);
                else
                    disp('ERROR: No contours found!');
                    return;
                end

                [contours, ~] = obj.getContours(filteredFrame);
                areas = [];
                % Iterate over contours
                for i = 1:min(contours.NumObjects, numel(contours.PixelIdxList))
                    % Access the i-th element of the PixelIdxList field
                    contour = contours.PixelIdxList{i};
                    
                    % Convert the linear indices to subscript indices
                    [rows, cols] = ind2sub(size(filteredFrame), contour);
                    
                    % Calculate the area of the contour
                    area = polyarea(cols, rows);
                    
                    % Append the area to the areas array
                    areas = [areas; area];
                end
                areas
                if isempty(areas)
                    disp("Error! Could not find any object!");
                elseif length(areas) > 1
                    disp("Error! More than one object identified!");
                else
                    % Check area value to identify box size
                    if 0 <= areas(1) && areas(1) < 180
                        disp("Caixa Pequena!");
                        boxToPick = 1;
                    elseif 180 <= areas(1) && areas(1) < 340
                        disp("Caixa grande!");
                        boxToPick = 2;
                    else
                        disp(['Error! Area does not match expected value: ', num2str(areas(1))]);
                    end
                end
            catch ME
                disp(['Error processing frame: ', ME.message]);
            end   
        end
    end
    methods (Access = private)
        function [contours, frameSize] = getContours(~, frame)
            try
                % Crop the frame to the left half
                gray_filt = rgb2gray(frame); % Convert the image to grayscale
                blur_filt = imgaussfilt(gray_filt, 5);
                th1 = imbinarize(blur_filt, 30/255);  % Add threshold to remove background noise
                gradient_filt = imgradient(th1); % Filter to show only contours
                imshow(gradient_filt)
                frameSize = size(gradient_filt);
                contours = bwconncomp(gradient_filt);
            catch ME
                disp(['Error processing frame in getContours: ', ME.message]);
            end
        end
    end
end

