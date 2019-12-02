function[]=chainer(n,m,r,varargin)
% function[]=chainer(n,r,'option')
%INPUT
%n = number of sides (even and n>2)
%r = hole radius [m]
%ny=1 (default):
%OPTION:
% 'r' : bigger radius = 0.096 [m]
% 'l' : bigger length = 0.300 [m]
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%da cencellare:RAM
if n*m>32*32
   error('n_max=32')
end


c = 0.269;   % nfl football length (twice semilatus rectum)
p = 0.076;   % nfl football radius (periapse)
AR=[];

if ~isempty(varargin)

    AR=varargin{1};

    if strcmp(AR,'r')

        if rem(n,2)~=0 || n<=2 || r>=0.09
           error('sides n>2 even, radius r<0.09,   r_tot=0.096')
        end

        p = 0.096;  
       
    elseif strcmp(AR,'l')
      
        if rem(n,2)~=0 || n<=2  || r>=0.07
           error('sides n>2 even, radius r<0.07,   r_tot=0.076')
        end
       
        c = 0.300;
    else
        error('option unknow. Possible choise ''r'' or ''l''') 
    end

else
    if rem(n,2)~=0 || n<=2 || r>=0.07
       error('sides n>2 even, radius  r<0.07,   r_tot=0.076')
    end

end

cd aircraft
try 
   load CHAINWING
end

%ny=input('number of panels per partition (lato), ny= ');
ny = 1; %panels element spanwise
nx = m; %panels chordwise

nelem = n/2;
teta = 2*pi/n;
l = 2*r*sin(pi/n);  % partition span (length chain element) 


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
geo.p	     = p		       ;  % periapse
geo.semi     = c/2     	       ;  % semilatus
geo.c	     = c		       ;


ecce = c/2/p-1             % eccentricity
semi_corda = c/2/sqrt(1-ecce^2)*sqrt(1-((r+ecce*c/2/(1-ecce^2))/(c/2/(1-ecce^2)))^2)
 
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

if isempty(AR)
   
    save CHAINWING.mat geo tor_version

else

    if strcmp(AR,'r')
        save CHAINWING_r.mat geo tor_version 
    else   
        save CHAINWING_l.mat geo tor_version    
    end

end

clear

cd ..

end

