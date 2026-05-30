clear; clc; close all;

% Definiera vilka två filer som ska köras (med dina exakta filnamn)
datasets = {'intel.gfs.log', 'fr-campus-20040714.carmen.gfs.log'};

for d = 1:length(datasets)
    filename = datasets{d};
    fprintf('\nStartar processering av: %s\n', filename);
    
    if contains(filename, 'intel')
        % Inställningar för Intel Lab (Inomhus)
        resolution = 10;  % 10 celler per meter (1 cell = 10 cm)
        map_width = 1000;  
        map_height = 1000;
        offset_x = map_width / 2;
        offset_y = map_height / 2;
        fig_name = 'Occupancy Grid Map - Intel Lab (Inomhus)';
        max_trust_range = 20.5; 
        data_scale = 1.0; % Intel är redan i korrekta enheter
    else
        % Inställningar för Freiburg Campus (Utomhus)
        % KORRIGERING: Vi sänker upplösningen till 2 celler per meter (1 cell = 50 cm).
        % Detta gör varje prick 5 gånger större och tjockare i matrisen!
        resolution = 5;  
        map_width = 3000;  
        map_height = 3000;
        offset_x = map_width / 2; 
        offset_y = map_height / 2;
        fig_name = 'Occupancy Grid Map - Freiburg Campus (Utomhus)';
        max_trust_range = 20.0; 
        
        % KORRIGERING: Skalningsfaktor för att konvertera laserenheterna i Freiburg
        data_scale = 1; % Testa 0.01 om datan är i cm, eller behåll 1.0 om den är i meter
    end
    
    % Initiera kartan med nollor (Log-Odds = 0 betyder sannolikhet 0.5, dvs helt okänt)
    map_log_odds = zeros(map_height, map_width);
    
    % Öppna den korrigerade carmen-loggen
    fid = fopen(filename, 'r');
    if fid == -1
        warning('Kunde inte öppna filen: %s. Kontrollera att den ligger i rätt mapp!', filename);
        continue;
    end
    
    % Skapa en unik figur för detta dataset
    figure('Name', fig_name);
    colormap(gray); 
    
    count = 0;
    while ~feof(fid)
        line = fgetl(fid);
        
        if startsWith(line, 'FLASER')
            data = strsplit(line);
            num_readings = str2double(data{2});
            
            % 1. Plocka ut lasermätningar och applicera skalningsfaktorn
            laser_ranges = str2double(data(3 : 2 + num_readings)) * data_scale;
            
            % 2. Plocka ut robotens sanna korrigerade position
            robot_x = str2double(data{3 + num_readings});
            robot_y = str2double(data{4 + num_readings});
            robot_theta = str2double(data{5 + num_readings});
            
            % 3. Uppdatera kartan direkt (Inkrementellt)
            map_log_odds = update_occupancy_grid(map_log_odds, robot_x, robot_y, robot_theta, laser_ranges, resolution, offset_x, offset_y, max_trust_range);
            
            % 4. Rita ut kartan var 100:e rad för att spara datorkraft
            count = count + 1;
            if mod(count, 100) == 0
                imagesc(flipud(1 - 1./(1+exp(map_log_odds)))); 
                axis equal;
                set(gca, 'YDir', 'reverse'); 
                title(sprintf('Genererar %s... Tidssteg: %d', filename, count));
                drawnow;
            end
        end
    end
    fclose(fid);
    
    % Slutgiltig rendering av denna karta när hela loggfilen är läst
    imagesc(flipud(1 - 1./(1+exp(map_log_odds))));
    axis equal;
    set(gca, 'YDir', 'reverse');
    title(['Färdig karta (Perfekt matchning): ', filename]);
    
    % Spara undan matrisen i workspace med unikt namn inför Path Planning-steget
    if contains(filename, 'intel')
        intel_map = map_log_odds;
    else
        freiburg_map = map_log_odds;
    end
end
fprintf('\nBåda kartorna har genererats framgångsrikt!\n');


% =========================================================================
% LOKALA FUNKTIONER
% =========================================================================

function map = update_occupancy_grid(map, rx, ry, rtheta, ranges, res, off_x, off_y, max_trust_range)
    l_occ = 1.2;    % Höjt något för att göra väggar/prickar ännu tydligare
    l_free = -0.5; 
    
    % Robotens cell-koordinater i matrisen
    robot_cell_x = round(rx * res) + off_x;
    robot_cell_y = round(ry * res) + off_y;
    
    num_angles = length(ranges);
    
    % --- DYNAMISKT OCH KORRIGERAT VINKELSVEP ---
    if num_angles <= 181
        angles = linspace(-pi/2, pi/2, num_angles);
    else
        angles = linspace(-pi, pi, num_angles); 
    end
    
    % Loopa igenom varje enskild laserstråle
    for i = 1:num_angles
        r = ranges(i);
        
        if r > max_trust_range || r < 0.1
            continue;
        end
        
        % Strålens absolut vinkel i världen
        if num_angles <= 181
            global_angle = rtheta + angles(i);
        else
            global_angle = rtheta - angles(i); 
        end
        
        % Hindrets position i verkligheten (meter)
        hinder_x = rx + r * cos(global_angle);
        hinder_y = ry + r * sin(global_angle);
        
        % Hindrets cell-koordinater i matrisen
        hinder_cell_x = round(hinder_x * res) + off_x;
        hinder_cell_y = round(hinder_y * res) + off_y;
        
        % --- Anrop till Bresenham ---
        [X, Y] = bresenham(robot_cell_x, robot_cell_y, hinder_cell_x, hinder_cell_y);
        
        % Celler längs strålen sätts till FREE (Svart)
        for j = 1:(length(X)-1)
            if X(j) > 0 && X(j) <= size(map, 2) && Y(j) > 0 && Y(j) <= size(map, 1)
                map(Y(j), X(j)) = map(Y(j), X(j)) + l_free;
            end
        end
        
        % Sista cellen där lasern studsade sätts till OCCUPIED (Vit)
        if hinder_cell_x > 0 && hinder_cell_x <= size(map, 2) && hinder_cell_y > 0 && hinder_cell_y <= size(map, 1)
            map(hinder_cell_y, hinder_cell_x) = map(hinder_cell_y, hinder_cell_x) + l_occ;
        end
    end
end

function [x, y] = bresenham(x1, y1, x2, y2)
    dx = abs(x2 - x1); dy = abs(y2 - y1);
    sx = sign(x2 - x1); sy = sign(y2 - y1);
    x = x1; y = y1;
    if dx > dy
        err = dx / 2;
        while x(end) ~= x2
            x = [x; x(end) + sx];
            err = err - dy;
            if err < 0
                y = [y; y(end) + sy];
                err = err + dx;
            else
                y = [y; y(end)];
            end
        end
    else
        err = dy / 2;
        while y(end) ~= y2
            y = [y; y(end) + sy];
            err = err - dx;
            if err < 0
                x = [x; x(end) + sx];
                err = err + dy;
            else
                x = [x; x(end)];
            end
        end
    end
end