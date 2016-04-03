% function circ = DrawCircle(radius)
%radius is in pixels;
radius = 4;
for i = length(radius)
    checLocations = zeros(radius*radius);
    x = i+1;
    y0 = 1;
    x0 = 1;
    y = 1;
    decisionOver2 = 1 - x;   %// Decision criterion divided by 2 evaluated at x=r, y=0
    
    while(y <= x)
        checLocations( x + x0,  y + y0) = 1; % // Octant 1
        checLocations( y + x0,  x + y0) = 1; %// Octant 2
        checLocations(-x + x0,  y + y0) = 1; %// Octant 4
        checLocations(-y + x0,  x + y0) = 1; %// Octant 3
        checLocations(-x + x0, -y + y0) = 1; %// Octant 5
        checLocations(-y + x0, -x + y0) = 1; %// Octant 6
        checLocations( x + x0, -y + y0) = 1; %// Octant 7
        checLocations( y + x0, -x + y0) = 1; %// Octant 8
        y = y + 1;
        if (decisionOver2<=0)
            decisionOver2 = decisionOver2 + 2 * y + 1; %  // Change in decision criterion for y -> y+1
            
        else
            x = x -1;
            decisionOver2 = decisionOver2 + 2 * (y - x) + 1;  % // Change for y -> y+1, x -> x-1
        end
    end
end