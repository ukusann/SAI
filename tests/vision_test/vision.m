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

            % canal_vermelho = image1(:,:,1);
            % canal_verde = image1(:,:,2);
            % canal_azul = image1(:,:,3);
            % 
            % limite_inferior = 245;
            % limite_superior = 255;
            % 
            % mascara_vermelha = (canal_vermelho >= limite_inferior) & (canal_vermelho <= limite_superior);
            % img_vermelho = image1.* uint8(mascara_vermelha);
            % %
            % % % VERDE
            % mascara_verde = (canal_verde >= limite_inferior) & (canal_verde <= limite_superior);
            % img_verde = image1.* uint8(mascara_verde);

            % %% AZUL
            % mascara_azul = (canal_azul >= limite_inferior) & (canal_azul <= limite_superior);
            % img_azul = image1.* uint8(mascara_azul);
            % %
            % img_vermelho_final = img_vermelho - img_azul - img_verde;
            % 
            % img_azul_final = img_azul - img_vermelho - img_verde;
            % 
            % img_verde_final = img_verde - img_vermelho - img_azul;
            % %
            % %
            % 
            % 
            % %Deteção objetos vermelhos
            % img_vermelho_final =  rgb2gray(img_vermelho_final);
            % img_vermelho_final = imbinarize(img_vermelho_final,graythresh(img_vermelho_final));
            % 
            % se = strel('square',8);
            % 
            % img_vermelho_final = imdilate(imerode(img_vermelho_final,se),se);
            % 
            % img_vermelho_final = imfill(img_vermelho_final,"holes"); %Preenche de branco os objetos
            % figure, imshow(img_vermelho_final);
            % 
            % %deteção coordenadas vermelho
            % figure;
            % props_vermelho = regionprops(img_vermelho_final, 'Centroid');
            % % Imagem original para verde
            % imshow(img_vermelho_final);
            % title('Objetos Vermelhos');

            % %Adicionar coordenadas para objetos vermelho
            % hold on;
            % for k = 1:length(props_vermelho)
            %     centroid = props_vermelho(k).Centroid
            %     scatter(centroid(1), centroid(2), 'red', 'filled');
            %     texto_legenda = sprintf('(%d, %d)', centroid(1), centroid(2));
            %     text(centroid(1) + 5,centroid(2) + 5, texto_legenda, 'Color', 'r', 'FontSize', 10);
            % end
           
            % props_vermelho = regionprops(img_vermelho_final, 'Centroid');

            %Adicionar retângulos para objetos vermelhos
            % figure;

            %Imagem original para vermelho
            %  imshow(img_vermelho_final);
            %  title('Objetos Vermelhos');
            % 
            % %Adicionar retângulos para objetos verdes
            % hold on;
            % for k = 1:length(props_vermelho)
            %     centroid = props_vermelho(k).Centroid
            %      rectangle('Position', [centroid(1)-20, centroid(2)-20, 40, 40 ], 'EdgeColor', 'r', 'LineWidth', 2);
            % end
            % 
            % hold off;







            %------------------------------------------Visão por computador---------

            % [res, camhandle] = obj.vrep.simxGetObjectHandle(obj.clientID, 'Vision_sensor', obj.vrep.simx_opmode_oneshot_wait);
            %
            %
            % [res, resolution, image1] = obj.vrep.simxGetVisionSensorImage2(obj.clientID, camhandle, 0, obj.vrep.simx_opmode_oneshot_wait);
            %
            % img=uint8(image1);
            % imshow(image1);


            % %Segmentação em 3 cores
            % numColors = 3;
            % L = imsegkmeans(image1,numColors);
            %
            % B = labeloverlay(image1, L);
            % figure, imshow(B); %resultado da segmentação
            %
            % %CONVERSÃO DA IMAGEM DE RGB PARA LAB
            % lab_image1 = rgb2lab(image1); %Conversão de RGB para LAB
            %
            % ab = lab_image1(:,:,2:3); %ab fica com os valores de a e b da imagem em LAB
            % ab = im2single(ab); % todos os valores ficam entre 0 e 1
            % pixel_labels = imsegkmeans(ab, numColors, NumAttempts=3); % segmentação dividindo em 3 clusters com 3 iterações
            % %obtêm-se assim os valores dos pixeis das 3 cores
            %
            % figure, imshow(labeloverlay(image1, pixel_labels));
            % %----------------------------------cluster vermelho------------------------------------------------------
            %
            % mask1 = pixel_labels == 1; % todos os pixeis com componente vermelha
            % cluster1 = image1 .* uint8(mask1); %filtragem usando a máscara
            %
            % figure, imshow(cluster1);
            % %---------------
            % L = lab_image1(:,:,1); % guardar os valores de luminosidade da imagem
            % L_red = L .* double(mask1); % filtrar apenas os vermelhos
            % L_red = rescale(L_red); % converter numa escala de 0 a 1
            %
            % idx_light_red = imbinarize(nonzeros(L_red)); % encontrar todos os valores que não zero e passar para valores
            % red_idx = find(mask1); % encontrar todos os zeros
            % mask_dark_red = mask1;
            % mask_dark_red(red_idx(idx_light_red)) = 0; % todos os vermelhos claros são 0.
            %
            % red_image1 = image1 .* uint8(mask_dark_red); %filtrar tudo o que não é vermelho
            % figure, imshow(red_image1);
            % %--------------------------------
            % red_image1 =  rgb2gray( red_image1 ); %Passar para a escala de cinzentos
            %
            % red_image1 = imbinarize(red_image1,graythresh(red_image1)); %Filtragem dos cizentos com 0.35 de threshold
            %
            % red_image1 = edge(red_image1,"sobel");
            %
            % figure, imshow(red_image1);
            %
            %
            % %-----------------------------cluster Azul----------------------
            % mask2 = pixel_labels == 2; % todos os pixeis com componente azul
            % cluster2 = image1 .* uint8(mask2); %filtragem usando a máscara
            %
            % figure, imshow(cluster2);
            %
            % %------------------
            % blue_image1 =  rgb2gray(cluster2); %Passar para a escala de cinzentos
            %
            % blue_image1 = imbinarize(blue_image1,graythresh(blue_image1)); %Filtragem dos cizentos
            %
            % blue_image1 = edge(blue_image1,"sobel"); %Encontra as bordas dos objetos
            %
            %
            % figure, imshow(blue_image1);
            %
            % %------------------------cluster verde---------------
            % mask3 = pixel_labels == 3;% todos os pixeis com componente verde
            % cluster3 = image1 .* uint8(mask3); %filtragem usando a máscara
            %
            % figure, imshow(cluster3);
            % %------------
            % green_image1 =  rgb2gray(cluster3); %Passar para a escala de cinzentos
            %
            % green_image1 = imbinarize(green_image1,graythresh(green_image1)-0.1);  %Filtragem dos cizentos
            %
            % green_image1 = edge(green_image1,"sobel");
            %
            % figure, imshow(green_image1);
            %
            % %--------------------------Centroides verde--------------
            %
            % props_verde = regionprops(green_image1, 'Centroid');
            %
            % %Adicionar retângulos para objetos verdes
            % %figure;
            %
            % %Imagem original para verde
            % imshow(green_image1);
            % title('Objetos Verdes');
            %
            % %Adicionar retângulos para objetos verdes
            % hold on;
            % for k = 1:length(props_verde)
            %     centroid = props_verde(k).Centroid
            %     rectangle('Position', [centroid(1)-50, centroid(2)-50, 100, 100], 'EdgeColor', 'g', 'LineWidth', 2);
            % end
            %
            % %----------------------------Centroides Azuis--------------
            % props_azul = regionprops(blue_image1, 'Centroid');
            %
            % %Adicionar retângulos para objetos azuis
            % figure;
            %
            % %Imagem original para azuis
            % imshow(blue_image1);
            % title('Objetos Azuis');
            %
            % %Adicionar retângulos para objetos azuis
            % hold on;
            % for k = 1:length(props_azul)
            %     centroid = props_azul(k).Centroid
            %     rectangle('Position', [centroid(1)-50, centroid(2)-50, 100, 100], 'EdgeColor', 'b', 'LineWidth', 2);
            % end
            %
            % hold off
            %
            % %---------------------Centroides Vermelhos---------------
            % props_vermelho = regionprops(red_image1, 'Centroid');
            %
            % %Adicionar retângulos para objetos vermelhos
            % figure;
            %
            % %Imagem original para vermelho
            % imshow(red_image1);
            % title('Objetos Vermelhos');
            %
            % %Adicionar retângulos para objetos verdes
            % hold on;
            % for k = 1:length(props_vermelho)
            %     centroid = props_vermelho(k).Centroid
            %      rectangle('Position', [centroid(1)-50, centroid(2)-50, 100, 100], 'EdgeColor', 'r', 'LineWidth', 2);
            % end
            %
            % hold off;
            %
            %
            % %---------------------------------------------------------------------------------------







        end
    end
end




% %- parte mais recente
% canal_vermelho = image1(:,:,1);
% canal_verde = image1(:,:,2);
% canal_azul = image1(:,:,3);
%
% limite_inferior = 245;
% limite_superior = 255;
%
% mascara_vermelha = (canal_vermelho >= limite_inferior) & (canal_vermelho <= limite_superior);
% img_vermelho = image1.* uint8(mascara_vermelha);
%
% % VERDE
% mascara_verde = (canal_verde >= limite_inferior) & (canal_verde <= limite_superior);
% img_verde = image1.* uint8(mascara_verde);
%
% %% AZUL
% mascara_azul = (canal_azul >= limite_inferior) & (canal_azul <= limite_superior);
% img_azul = image1.* uint8(mascara_azul);
%
% img_vermelho_final = img_vermelho - img_azul - img_verde;
%
% img_azul_final = img_azul - img_vermelho - img_verde;
%
% img_verde_final = img_verde - img_vermelho - img_azul;
%
%
%
% img_vermelho_final =  rgb2gray(img_vermelho_final);
% img_vermelho_final = imbinarize(img_vermelho_final,graythresh(img_vermelho_final));
%
% se = strel('square',8);
%
% img_vermelho_final = imdilate(imerode(img_vermelho_final,se),se);
%
% img_vermelho_final = imfill(img_vermelho_final,"holes"); %Preenche de branco os objetos
% figure, imshow(img_vermelho_final);




