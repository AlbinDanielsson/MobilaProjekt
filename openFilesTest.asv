lines = readlines('Intel/intel.log');

% Remove comments and empty lines
lines = lines(~startsWith(lines, "#"));
lines = lines(strlength(lines) > 0);

odomLines   = lines(startsWith(lines, "ODOM"));
flaserLines = lines(startsWith(lines, "FLASER"));
paramLines  = lines(startsWith(lines, "PARAM"));