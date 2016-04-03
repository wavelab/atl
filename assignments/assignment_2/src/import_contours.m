function polygons = import_contours(filename)

fid = fopen(filename);
tline = fgetl(fid);
num_polys = 1;
polygons = cell(1);
curline = 0;
while ischar(tline)
    polysize = str2double(tline);
    tempcontour = nan(polysize,2);
    for i = 1:polysize
        tline = fgetl(fid);
        tmp = textscan(tline, '%f %f', 'Delimiter', ',');
        tempcontour(i,:) = [tmp{1}, tmp{2}];
    end
    polygons{num_polys,1} = tempcontour;
    num_polys = num_polys + 1; 
    tline = fgets(fid);
end
fclose(fid)