function [Im] = sawtoothNormalMap(twidth1,twidth2,n1,n2,size)
%SAWTOOTHNORMALMAP Generates a normal map with sawtooth pattern.
%
width = size(1);
height = size(2);

xi = 1;
pixels = zeros(3, width);
while xi < width
    xstart = xi;
    xend = min(width, xstart+twidth1-1);   
    pixels(:, xstart:xend) = repmat((n1 + 1.0) / 2.0, [1 (xend-xstart+1)]);
    if( xend == width )
        break;
    end
        
    xi = xend+1;
    xstart = xi;
    xend = min(width, xstart+twidth2-1);   
    pixels(:, xstart:xend) = repmat((n2 + 1.0) / 2.0, [1 (xend-xstart+1)]);

    xi = xend+1;
end

Im = repmat(pixels', [height 1 1]);
Im = reshape(Im, [width height 3]);
end

