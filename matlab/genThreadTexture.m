px = 2^11; %2^11;
img = zeros( px, px, 3, 'uint8' );

% let's put the thread pitch into the image.

% x dimension will be the rotation
% y dimension will be the distance along

% use NN lookup to the thread profile function.


% let's aim for M4 metric screw
% https://en.wikipedia.org/wiki/ISO_metric_screw_thread

diameter = 4;  % 4 mm
P = 0.7;       % one thread per 0.7 mm

% P*1/8 flat bit on outside
% P*2/8 flat bit on inside
% leave 5/8 for the sloped parts... 2.5/8*P length each

%  A1 A2 A3 A4
%           _      outer edge
%   \      /
%    \    /
%     \__/         inner edge
%
%
%  ------------    screw axis
%
A1 = 0;
A2 = 2.5/8*P;
A3 = 4.5/8*P;
A4 = 7/8*P;
A5 = P;

length = 20;           % 20 mm long

width = diameter * pi; % circumference for M4 screw

hyp = sqrt(P^2 + width^2);
c = P/hyp;
s = width/hyp;

for x = 0:(px-1)
    for y = 0:(px-1)
        a = mod( x/px*P + y/px*length, P ); % how far along the thread profile
        if ( a < A2 )
            n = [c,s*-sind(60),cosd(60)];            
        elseif ( a < A3 )
            n = [0,0,1]; % flat
        elseif ( a < A4 )
            n = [-c,s*sind(60),cosd(60)];            
        else
            n = [0,0,1]; % flat
        end
        img(y+1,x+1,:) = uint8( (n + [1,1,1] / 2) * 255 );        
    end
end
figure(1)
imshow(img)
imwrite( img, '../resources/threadM4-20.png', 'png' );