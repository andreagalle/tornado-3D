%volume nfl foro 0.045
ecce= .7697;
d= .0891; % lungezzza foro(dovuta al raggio del foro)
a=.1345/(1-ecce^2) %maggiore
b=.1345/sqrt(1-ecce^2) % minore
c=b-.0760 % distanza focale

Vo=2*pi*(a^2/b^2*((2*b^2-c^2)*d-1/3*d^3)-a^2*b*sqrt(1-(c/b)^2)*(asin(d/b)+d/b*sqrt(1-(d/b)^2))...
    -d*a^2/b^2*(sqrt(b^2-d^2)-sqrt(b^2-c^2))^2) % 0.001347
