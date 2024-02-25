function [Im] = constNormalMap(n,size)
%CONSTNORMALMAP Generates a normal map with sine wave pattern.
%
width = size(1);
height = size(2);
Im = zeros(height, width, 3);

for yi = 1:height
    for xi = 1:width
        Im(yi, xi, :) = (n + 1.0) / 2.0; 
    end
end

end

