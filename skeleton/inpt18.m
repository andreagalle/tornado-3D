
function [geo]=inpt15(geo)

%settings=config('startup'); %setting directories

 geo=[];

 geo.flapped=0;    % Is partition flapped [1 0]

% this is just to init the struct ?

 geo.nwing=1;      % Number of Wings

 geo.nelem(1)=10   % Number of semispanwise partitions for this wing

 geo.CG(1)=0;   % Center of gravity x-coordinate
 geo.CG(2)=0;   % Center of gravity y-coordinate
 geo.CG(3)=0;   % Center of gravity z-coordinate

 geo.ref_point(1)=0;   % Reference point x-coordinate
 geo.ref_point(2)=0;   % Reference point y-coordinate
 geo.ref_point(3)=0;   % Reference point z-coordinate

 geo.symetric(s)=0;    % Is the wing mirrored in the xz-plane [1 0]

 geo.startx(1)=0;      % Apex x-coordinate
 geo.starty(1)=0;      % Apex y-coordinate
 geo.startz(1)=0;      % Apex z-coordinate

 geo.c(1)=1;  % Root chord

 geo.foil(1,:,1)={'0012'};   % Base chord airfoil NACA four digits - 0 flat plate

 geo.nx(1,1)=5;   % Number of panels chord wise

 geo.TW(1,1,1)=0; % First wing base chord is reference for twist [rad] 

 geo.dihed(1,:)=0; % Partition dihedral [rad] (angle?)

 geo.ny(1,:)=10; % Number of panels semi-span wise

 geo.b(1,:)=1;   % Span of partition
 geo.T(1,:)=1;   % Taper ratio

 geo.foil(1,:,2)={'0012'};   % Tip chord airfoil NACA four digits - 0 flat plate

 geo.SW(1,:)=0;   % Quarter chord line sweep [rad]
 geo.TW(1,:,2)=0; % Outboard twist [rad]

 geo.meshtype(1,:)=1;  % mesh distribution types: 1 Linear 2 Span half-cosine 
                       % 3 Span half-cosine chord cosine 5 Span cosine
                       % 6 chord cosine

 geo.flapped(1,:)=0;   % flapped partition [1 0]

% from now on (without any flap) they have to be zeroed anyway

 geo.fc(1,:)=0;     % Flap chord in fraction of local chord

 geo.fnx(1,:)=0;    % Number of chord wise panels on flap

 geo.fsym(1,:)=0;   % control surfaces symmetric deflect [1 0]

 geo.TW(1,:,1)=0; % Outboard twist [rad]

 geo.flap_vector=zeros(size(geo.flapped));   % to eventually deflect flaps

