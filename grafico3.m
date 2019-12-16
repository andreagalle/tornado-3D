% Lattice plot
cd output
load test1-Cx.mat
cd ..

color =zeros(1764,5); 
j=figure(33)
fill3(lattice.XYZ(:,:,1)',lattice.XYZ(:,:,2)',lattice.XYZ(:,:,3)',color')

axis equal
axis off
set(j,'color','white')
