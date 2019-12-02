function[]=chainer(n,m,rp,c,varargin)
% function[]=chainer(n,m,rp,ecce)
%INPUT:
%n    = number of sides=panels spanwise=partition spanwise (even and n>2)
%m    = number of panels chordwise
%rp   = max football radius [m]
%ecce = eccentricity (ellipse)
%ny   = 1 (default) partition panels spanwise
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%da cencellare:RAM
%if n*m>32*32
%   error('n_max=32')
%end


%c  = 0.269 ;   % nfl football length (twice semilatus rectum)
%rp = 0.076 ;   % nfl football radius (periapse)
%e  = 0.7697;
%AR=[];


if rem(n,2)~=0 || n<=2 
   error('sides n>2 even,')
end


semilatus = c/2			    ;
%semilatus = rp*(1+ecce)		    ;       
ecce      = semilatus/rp-1          ; %eccentricity
a         = semilatus/(1-ecce^2)    ; %semi-axis major
b	  = semilatus/sqrt(1-ecce^2); %semi-axis minor
%d= 0.0891; nfl r=0.045
%c = a-rp; % Focal dinstance

%CONSTRAIN_NODE__________________________________________________________________________

f=@(d)2*pi*(a^2/b^2*((2*b^2-semilatus^2)*d-1/3*d.^3)-a^2*b*sqrt(1-(semilatus/b)^2).*(asin(d/b)+d/b.*sqrt(1-(d/b).^2))...
     -d*a^2/b^2.*(sqrt(b^2-d.^2)-sqrt(b^2-semilatus^2)).^2)-0.001347000000;

iter = 0     	    ;  
h    = 0.001 	    ;
err  = 10^-15	    ;
ERR  = 100   	    ;
D    = [0 semilatus];
tic          	    ;

while ERR>err && toc<300

	iter = iter+1		       ;   
	d    = (D(1):h:D(2))	       ; 
	F    = f(d)		       ; 
	pos  = find(abs(diff(sign(F)))); 

	if length(pos)~=1
	    disp('nessuna o  più radici')
	end

	D   = [d(pos) d(pos+1)];
	ERR = abs(diff(D))     ;
	h   = h/1000	       ;
end

target = sum(D)/2;
%fprintf('precisione volume %f10.10\n',f(target));
%_______________________________________________________________________________________
                                         
d          = target  			              ;        
semi_corda = d				              ; 
r          = a/b*(sqrt(b^2-d^2)-sqrt(b^2-semilatus^2));  % hole radius

disp(' ')
disp([' Hole radius : ',num2str(r),' [m]'])
disp([' Chord :       ',num2str(d*2),' [m]'])

 
cd aircraft
try 
   load FOOTBALL
end

%ny=input('number of panels per partition (lato), ny= ');
ny = 1; %panels element spanwise
nx = m; %panels chordwise

nelem = n/2          ;
teta  = 2*pi/n	     ;
l     = 2*r*sin(pi/n);  % partition span (length chain element) 


if r==0
    geo.bit = 2;
else
    geo.bit = 1;
end




geo.flapped  = zeros(1,nelem)  ;
geo.nelem    = nelem           ;
geo.nwing    = 1               ;
geo.symetric = 1               ;
geo.startx   = 0               ;
geo.starty   = 0               ;
geo.startz   = 0               ;
geo.nx       = ones(1,nelem)*nx;  % panels chordwise matrix
geo.ny       = ones(1,nelem)*ny;  % panels partition spanwise matrix
geo.b        = ones(1,nelem)*l ;  % partition span matrix 
geo.raggio   = r               ;  % hole radius
geo.p	     = rp	       ;  % periapse
geo.semi     = semilatus       ;  % semilatus
geo.c	     = c	       ;
geo.e	     = ecce	       ;

%ecce = c/2/rp-1;             % eccentricity
%semi_corda = c/2/sqrt(1-ecce^2)*sqrt(1-((r+ecce*c/2/(1-ecce^2))/(c/2/(1-ecce^2)))^2);
 
geo.CG        = [semi_corda,0,r];
geo.ref_point = [0 0 0]         ;



geo.TW          = zeros(1,nelem,2);
geo.foil        = cell(2,nelem,2) ;
geo.dihed       = zeros(1,nelem)  ;
geo.T           = zeros(1,nelem)  ;
geo.SW          = zeros(1,nelem)  ;
geo.meshtype    = zeros(1,nelem)  ;

geo.fc          = zeros(1,nelem)  ;
geo.fnx         = zeros(1,nelem)  ;
geo.fsym        = zeros(1,nelem)  ;
geo.flap_vector = zeros(1,nelem)  ;

teta_0 = pi/2-(pi-teta)/2;

for i = 1:nelem

    
    geo.foil       (1,i,:) = {'football'};
    geo.foil	   (2,i,:) = {'0000'}	 ;
    geo.TW         (1,i,:) = 0           ;
    geo.dihed      (1,i)   = teta_0 	 ;
    geo.T          (1,i)   = 1     	 ;
    geo.SW         (1,i)   = 0     	 ;
    geo.meshtype   (1,i)   = 1     	 ;
                           
    geo.fc         (1,i)   = 0     	 ;
    geo.fnx        (1,i)   = 0     	 ;
    geo.fsym       (1,i)   = 0     	 ;
    geo.flap_vector(1,i)   = 0     	 ;
    
    teta_0 = teta_0+teta;
    
end


tor_version = 135;

save FOOTBALL.mat geo tor_version

clear

cd ..

end

