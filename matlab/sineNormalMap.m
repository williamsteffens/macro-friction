function [Im] = sineNormalMap(ax,ay,wx,wy,toff,size)
%SINENORMALMAP Generates a normal map with sine wave pattern.
%
width = size(1);
height = size(2);
Im = zeros(height, width, 3);

for yi = 1:height
    for xi = 1:width
        tx = (xi-1) / (width-1);
        ty = (yi-1) / (height-1);
        
        dx = ax * sin(wx*(tx+toff));
        dy = ay * sin(wy*(ty+toff));
        
        vec = [ dx dy 1 ];
        n = vec / norm(vec);
        Im(yi, xi, :) = (n + 1.0) / 2.0; 
    end
end

end

