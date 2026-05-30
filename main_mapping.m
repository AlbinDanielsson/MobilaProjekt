clear; clc; close all;

% Definiera vilka två filer som ska köras (med dina exakta filnamn)
datasets = {'intel.gfs.log', 'fr-campus-20040714.carmen.gfs.log'};

for d = 1:length(datasets)
    filename = datasets{d};
    fprintf('\nStartar processering av: %s\n', filename);
    
    % --- DYNAMISKA INSTÄLLNINGAR BASERAT PÅ MILJÖ ---
    resolution = 10;  % 10 celler per meter (1 cell = 10 cm)
    
    if contains(filename, 'intel')
        % Inställningar för Intel Lab (Inomhus)
        map_width = 1000;  
        map_height = 1000;
        offset_x = map_width / 2;
        offset_y = map_height / 2;
        fig_name = 'Occupancy Grid Map - Intel Lab (Inomhus)';
        
        % DYNAMISKT KRITERIUM: Lärarens rekommendation (1/4 av max sensoravstånd)
        % Intel max_range är ca 81.83m -> 81.83 / 4 = ca 20.5m
        max_trust_range = 20.5; 
    else
        % Inställningar för Freiburg Campus (Utomhus)
        map_width = 6000;  
        map_height = 6000;
        offset_x = map_width / 2 - 1000; 
        offset_y = map_height / 2 + 1000;
        fig_name = 'Occupancy Grid Map - Freiburg Campus (Utomhus)';
        
        % DYNAMISKT KRITERIUM: Justerbart utifrån lärarens rekommendation. 
        % Om 20.5m klipper för mycket utomhus kan du justera denna upp till ca 30-40m.
        max_trust_range = 40.0; 
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
            
            % 1. Plocka ut lasermätningar (avstånd i meter)
            laser_ranges = str2double(data(3 : 2 + num_readings));
            
            % 2. Plocka ut robotens sanna korrigerade position
            robot_x = str2double(data{3 + num_readings});
            robot_y = str2double(data{4 + num_readings});
            robot_theta = str2double(data{5 + num_readings});
            
            % 3. Uppdatera kartan direkt med den dynamiska max_trust_range
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
fprintf('\nBåda kartorna har genererats framgångsrikt med perfekt orientering!\n');