function [Im] = randomNormalMap(sigmax,sigmay,size)
%RANDOMNORMALMAP Generates a normal map from Guassian distribution
%
width = size(1);
height = size(2);
Im = zeros(height, width, 3);

for yi = 1:height    
        nx = normrnd(0, sigmax, [1 width]);
        ny = normrnd(0, sigmay, [1 height]);
        nz = ones([1 width]);
        N = [nx; ny; nz];
        N = normalize(N, 1, 'norm');
        Im(yi, :, :) = (N' + 1.0) / 2.0;
end

end

