%  Generates a texture with resolution of 1mm per pixel
%
%   Ra - Absolute roughness in mm
%   size - The texture size
function [Im] = texture1mm(Ra,size)

width = size(1);
height = size(2);
Im = zeros([height width 3]);

for yi = 1:height    
    theta = atan(Ra * rand([1 width]));
    phi = atan(Ra * rand([1 width]));
    
    nx = sin(theta);
    ny = sin(phi);
    nz = ones([1 width]);
    N = [nx; ny; nz];
    N = normalize(N, 1, 'norm');
    Im(yi, :, :) = (N' + 1.0) / 2.0;
end

