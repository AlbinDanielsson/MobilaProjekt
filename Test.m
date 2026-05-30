clc;
clear;
fid = fopen('intel.gfs.log', 'r');
while ~feof(fid)
    line = fgetl(fid); % Läs en rad
    
    if startsWith(line, 'FLASER')
        data = strsplit(line); % Dela upp raden vid varje mellanslag
        
        num_readings = str2double(data{2}); % Blir 180
        
        % 1. Plocka ut alla 180 lasermätningar
        laser_ranges = str2double(data(3 : 2 + num_readings));
        
        % 2. Plocka ut robotens position (ligger direkt efter lasermätningarna)
        robot_x = str2double(data{3 + num_readings});
        robot_y = str2double(data{4 + num_readings});
        robot_theta = str2double(data{5 + num_readings});
        
        % 3. Skicka in robot_x, robot_y, robot_theta och laser_ranges 
        % i din Occupancy Grid-funktion (Föreläsning sida 35)!
    end
end
fclose(fid);