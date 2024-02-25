% Testing out Eq. 3.74 from the "Friction Dynamics" book : 
% 
% mu = lambda * tan(  arcsin( (sqrt(2)*(2+psi)/(4*(1+psi) ) )
%

lambda = 1.0;  % portion of the plastically supported load
psi = 0:0.1:(pi/2); % microfacet angle

mu = lambda * tan(  asin( (sqrt(2)*(2+(pi/4)-psi))./(4*(1+(pi/2)-psi)) ) );
%mu = lambda * tan(  asin( (sqrt(2)*(2+psi))./(4*(1+psi)) ) );

plot(psi, mu);

