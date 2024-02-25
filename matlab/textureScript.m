width = 128;
height = 128;
T1 = im2double( imresize( imread('../resources/oak_normalmap.png'), [height width]) );
T2 = flipdim(T1,1);
%T1 = sineNormalMap(0, 1, 0, 20, 0.0, [width height]);
%T2 = sineNormalMap(0, 1, 0, 20, 0.5, [width height]);
%T1 = constNormalMap([0.7071; 0; 0.7071], [width height]);
%T2 = constNormalMap([-0.7071; 0; 0.7071], [width height]);
%T1 = sawtoothNormalMap(15, 1, [0; -0.7071; 0.7071], [0; 0.866; 0.5], [width height]);
%T2 = sawtoothNormalMap(15, 1, [0; 0.7071; 0.7071], [0; -0.866; 0.5], [width height]);
%T1 = randomNormalMap(1.0, 1.0, [width height]);
%T2 = randomNormalMap(1.0, 1.0, [width height]);




G1 = rgb2gray(T1);
G2 = rgb2gray(T2);
[Gmag1,Gdir1] = imgradient(G1);
[Gmag2,Gdir2] = imgradient(G2);

%W1 = stdfilt(rgb2gray((T1)));
%W2 = stdfilt(rgb2gray((T2)));
W1 = normalWeightMap(T1);
W2 = normalWeightMap(T2);
BW1 = edge(rgb2gray(T1));
BW2 = edge(rgb2gray(T2));

figure;
subplot(1,2,1);
imshow(W1);
subplot(1,2,2);
imshow(W2);


% Compute friction cone
R = [ 1 0 0; 0 1 0; 0 0 -1 ];   % reflect z axis
nmean = [0; 0; 1];
delta_ang = (pi/8);
nsamples = (2*pi)/delta_ang + 1;
points = zeros(nsamples,2);
coeffs = zeros(nsamples,1);
k = 1;
for ang = 0:delta_ang:(2*pi)
    v = [ cos(ang); sin(ang); 0 ];
    for xi = 1:width
       for yi = 1:height
           n1 = pixel2normal(T1(yi,xi,:));
           n2 = R*pixel2normal(T2(yi,xi,:));
           w1 = W1(yi,xi);
           w2 = W2(yi,xi);
           % flip normals in z-direction to create an opposing surface
           theta1 = acos(  max(0, min(1, n1'*nmean)) );
           theta2 = acos(  max(0, min(1, -n2'*nmean)) );
           mu1 = max(0, n1'*v) * (2/pi)*tan( min(1.57, theta1) );
           mu2 = max(0, -n2'*v) * (2/pi)*tan( min(1.57, theta2) );
           w1 = 1;
           w2 = 1;
           func = (w1*mu1 + w2*mu2);
           coeffs(k) = coeffs(k) + func;
       end
    end
    % add a small perturbation since otherwise we may create 
    % degenerate friction polygons.
    coeffs(k) = (coeffs(k) + 0.001*rand([1]))/(width*height);
    points(k,1) = coeffs(k) * v(1);
    points(k,2) = coeffs(k) * v(2);
    k = k + 1;
end

pgon = polyshape(points(:,1), points(:,2));

h = figure;
set(gcf,'Position',[100,100,1600,400])
subplot(1,3,1);
imagesc(T1);
title('texture_A');
set(gca,'XTick',[])
set(gca,'YTick',[])
subplot(1,3,2);
imagesc(T2);
title('texture_B');
set(gca,'XTick',[])
set(gca,'YTick',[])
subplot(1,3,3);
plot(pgon);
set(gca,'FontName','cmr12')
xlim([-2 2]);
xticks([-1 -0.5 0.0 0.5 1]);
ylim([-2 2]);
yticks([-1 -0.5 0.0 0.5 1]);
xlabel('$\vec{t}$','Interpreter','latex');
ylabel('$\vec{b}$','Interpreter','latex');

title('\mu');

