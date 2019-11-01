function[]=chainer(n,r)
% function[]=chainer(n,r)
%INPUT
%n= numero di lati (deve essere pari e maggiore di 2)
%r= raggio circonferanza in metri (nfl r=0.076)
%se non viene dato ny=2 (default):

if rem(n,2)~=0 || n<=2 || r<=0
    error('lati n>2 pari, raggio  r>0')
end

cd aircraft
load CHAINWING

ny=input('numero di pannelli lungo apertura elemento (lato), ny= ');
if isempty(ny) || ny==0
    ny=2;
else
    ny=abs(round(ny));
end

nelem=n/2;
teta=2*pi/n;
l=2*r*sin(pi/n);

geo.flapped=zeros(1,nelem);
geo.nelem=nelem;
geo.nx=ones(1,nelem)*20;
geo.ny=ones(1,nelem)*ny;
geo.b=ones(1,nelem)*l;

teta_0=pi/2-(pi-teta)/2;
for i=1:nelem
    
    geo.foil(1,i,:)={'naca4412.DAT'};
    geo.TW(1,i,:)=0;
    geo.dihed(1,i)=teta_0;
    geo.T(1,i)=1;
    geo.SW(1,i)=0;
    geo.meshtype(1,i)=1;
    
    geo.fc(1,i)=0;
    geo.fnx(1,i)=0;
    geo.fsym(1,i)=0;
    geo.flap_vector(1,i)=0;
    
    teta_0=teta_0+teta;
    
end

save CHAINWING.mat geo tor_version
clear
cd ..

end

