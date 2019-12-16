% Lattice plot 3D

load target.mat


color =zeros(1764*2,5); 
j=figure(33)
fill3(lattice.XYZ(:,:,1)',lattice.XYZ(:,:,2)',lattice.XYZ(:,:,3)',color')
colormap(winter)
axis equal
axis off
set(j,'color','white')