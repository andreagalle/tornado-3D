function [results]=solver(results,state,geo,lattice,ref)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (C) 1999, 2007 Tomas Melin
%
% This file is part of Tornado
%
% Tornado is free software; you can redistribute it and/or
% modify it under the terms of the GNU General Public
% License as published by the Free Software Foundation;
% either version 2, or (at your option) any later version.
%
% Tornado is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied
% warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
% PURPOSE.  See the GNU General Public License for more
% details.
%
% You should have received a copy of the GNU General Public
% License along with Tornado; see the file GNU GENERAL 
% PUBLIC LICENSE.TXT.  If not, write to the Free Software 
% Foundation, 59 Temple Place -Suite 330, Boston, MA
% 02111-1307, USA.
%
% usage: [RESULTS] = solver8(results,state,geo,lattice,ref)
%
% This function computes forces and moments on each panel.
% Inputs are coordinades for old resluts, collocationpoints, 
%   vorticies and Normals, reference area and chord
%
% Example:
%
%   [results]=solver8(results,state,geo,lattice,ref);
%
% Calls:
%           Setboundary    
%
% Author: Tomas Melin <melin@kth.se>
% Keywords: Tornado core function
%
% Revision History:
%   Bristol,  2007 06 27:  Addition of new header. TM.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%disp('Running solver 8')
[a vor_length void]=size(lattice.VORTEX);%extracting number of sections in 
	 									   %"horseshoes"
%a= num di pannelli
%vor_length = [wake TEP HP V HP TEP wake]= [1 1 1 2 1 1 1 ] colonne
% void =3

    %if vor_length < 8
    %   terror(1)
    %   return
    %end
    
%flops(0)    
[w2 void]=fastdw(lattice); % w2 è una matrice quadrata di scalari, void è trdimensionale
			   % w2 è la proiezione di void sulle rispettive normali

results.dwcond=cond(w2);
%disp('dnwash... ok')
%count=flops         
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Setting up right hand side %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
rhs=(setboundary5(lattice,state,geo))';  % ATTENZIONE:controllare se il trasposto va bene 
%disp('rhs... ok')
%gamma= inv(w2)*rhs';


try
   geo.raggio;
   %%%%%%%%%%%%%%%%%%%%%%%%%%
   %NEW Solver	%     %  %
   %%%%%%%%%%%%%%%%%%%%%%%%%%
   %%Gauss-Seidel_HIGH-DRIVE

   err  = 10        ;
   Xnew = zeros(a,1);
   tic   		  ;
  
   while any(find(err> 10^-8))
          
          Xold = Xnew;
  
          for i=1:a
             
              ci1 = 0;
              ci2 = 0;
          
              if i>1
                 ci1 = sum(w2(i,1:i-1)'.*Xnew(1:i-1));
              end
             
              ci2     = sum(w2(i,i+1:a)'.*Xold(i+1:a));
             
              Xnew(i) = 1/w2(i,i)*(-ci1-ci2+rhs(i))   ;
  
          end
  
          err = abs(Xnew-Xold);
  
          if toc>300
              disp(max(err));
          end
  
   end
  
  
   toc
   gamma = Xnew;
catch                              
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%
   %Solving for rhs           %
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%	
   % determina intensità vortice di ogni pannello
   gamma=w2\rhs';   % risolve il sistema x=A\B
   %disp('gauss... ok')
end
% righe di gamma= (nx*ny)*2 per simmetria (la parte simmetrica ha segno opposto),
% cioè le righe sono pari al numero di CONTROLLPOINT

% w2 =(nx*ny)*2 * (nx*ny)*2   quadrata 
%%%%%%%%%%%%%%%%%%%%%%%%%%
%Mgs     = tril(w2)                          ;
%Ngs     = triu(w2,1)                        ;
%Mgs_inv = inv(Mgs)                          ; 
%Cgs     = -Mgs_inv*Ngs                      ;
%Qgs     = Mgs_inv*rhs'                      ;
%gamma_0 = ones(size(rhs',1),size(rhs',2))   ;
%err     = 1			     	    ;
%gamma   = [] 				    ;

%if norm(Cgs,2)>1

%   disp('Il metodo potrebbe non convergere')

%end



%tic;
%while any(find(err>10^-6)) && toc<300

%     gamma   = Cgs*gamma_0+Qgs   ;
%     err     = abs(gamma-gamma_0);
%     gamma_0 = gamma	         ;

%end

%if toc>300
%  
%   disp('Il metodo non ha raggiunto la precisione richiesta 10^-6')

%end %New Solver
%%%%%%%%%%%%%%%%%%%%%%%%%%

% LU method

%[L U] = lu(w2);
%appo  = L\rhs'; 
%gamma = U\appo; 
%%%%%%%%%%%%%%%%%%%%%%%%%%               
               

    if state.pgcorr==1
        %tdisp('Trying PG correction')
        %Prandtl-Glauert compressibility correction
        [state.rho sos p_1]=ISAtmosphere(state.ALT);
        M=state.AS/sos;
        corr=1/(sqrt(1-M^2));  
        gamma=gamma*corr;
        %Yeah, this part is not validated yet... or even published, but it
        %seems to work. Do use it with caution though as the math rigour
        %isnt there yet.   
    end




b1=vor_length/2;
%estrae vortice 'a' e 'b' di ogni pannello
p1(:,:)=lattice.VORTEX(:,b1,:);		%Calculating panel vortex midpoint	
p2(:,:)=lattice.VORTEX(:,b1+1,:);	%to use as a force locus
lattice.COLLOC(:,:)=(p1+p2)./2;	    % LOCAL control point, vortex midpoint (usato e dimostrato il perchè)

c3=lattice.COLLOC-ones(size(lattice.COLLOC,1),1)*geo.ref_point; % ref_point è nullo

[w3 DW]=fastdw(lattice);	                    %Calculating downwash on vorticies
w4=sum(DW,2);					                %superpositioning aerodynamic influence

DWX=DW(:,:,1);
DWY=DW(:,:,2);
DWZ=DW(:,:,3);

[void nofderiv]=size(gamma);
le=(p2-p1);					%Vortex span vector


%for s=1:a
%	Lle(s)=norm(le(s,:));			%length of vortex span vector or panel span
%	lehat(s,:)=le(s,:)./Lle(s);	%
%end 

% 'le' è il vettore che rappresenta il filamento vorticoso al quarto di corda del generico pannello 
Lle=sqrt(sum(le.^2,2));
lehat(:,1)=le(:,1)./Lle;
lehat(:,2)=le(:,2)./Lle;
lehat(:,3)=le(:,3)./Lle;

% adesso noofderiv=1 , dipendeva dalle condizoni al contorno (serviva per diff finite ecc..)  
for j=1:nofderiv
    IW(:,j,1)=DWX*gamma(:,j);  % InducedWind=Downwash(direzione)*intensità di cirolazione  (GAMMA=m^2/s , gamma = m/s)
    IW(:,j,2)=DWY*gamma(:,j);
    IW(:,j,3)=DWZ*gamma(:,j);
    
    G(:,1)=gamma(:,j).*lehat(:,1);	% Aligning vorticity along panel vortex
    G(:,2)=gamma(:,j).*lehat(:,2);	
    G(:,3)=gamma(:,j).*lehat(:,3);

    wind1=state.AS*([cos(state.alpha)*cos(state.betha) -cos(state.alpha)*sin(state.betha) sin(state.alpha)]); %Aligning with wind

    for i=1:a
        Wind(i,:)=wind1-squeeze(IW(i,j,:))';
        Rot(i,:)=cross((lattice.COLLOC(i,:)-geo.CG),[state.P state.Q state.R]); %Calculating rotations
    end                                   %^^^^^^^---new stuff in T131         %Thanks Luca for pointing out the error here

    Wind=Wind+Rot;								%Adding rotations
    Fprim(:,j,:)=state.rho*cross(Wind,G);   %Force per unit length (per span unit) Kutta-Jukowsky (invertita per il segno)
    
    
        F(:,j,1)=Fprim(:,j,1).*Lle;				%Force per panel
        F(:,j,2)=Fprim(:,j,2).*Lle;				%Force per panel
        F(:,j,3)=Fprim(:,j,3).*Lle;				%Force per panel
    

    C3(:,:,1)=c3(:,1)*ones(1,nofderiv);
    C3(:,:,2)=c3(:,2)*ones(1,nofderiv); 
    C3(:,:,3)=c3(:,3)*ones(1,nofderiv); 
        
end

results.F=F;   % 3 matrici, componenti forze nelle tre direzioni su ogni pannello
results.FORCE=sum(F,1);			 %Total force (su tutto il velivolo)
M=cross(C3,F,3);	             %Moments per panel
results.M=M;
results.MOMENTS=sum(M,1);					%Summing up moments	
results.gamma=gamma;

end%FUNCTION

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
%
% NEW DOWNWASH FUNCTION
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5five
function[dw,DW]=fastdw(lattice)
one_by_four_pi=1/(4*pi);

%calcola le 'dw' specifiche per unità di vorticità gamma di ogni pannello dovute a tutti i vortici
%si può fare perchè tanto dipende solo geometria e dalla posizione dei vortici 
[psize vsize void]=size(lattice.VORTEX);


%disp('running right')
%psize=size(lattice.COLLOC,1);
lemma=ones(1,psize);   % lo usa per costruire matrici con simmetrie

LDW=zeros(psize,psize,7,3);

mCOLLOC(:,:,1)=lattice.COLLOC(:,1)*lemma;
mCOLLOC(:,:,2)=lattice.COLLOC(:,2)*lemma;
mCOLLOC(:,:,3)=lattice.COLLOC(:,3)*lemma;

mN(:,:,1)=lattice.N(:,1)*lemma;
mN(:,:,2)=lattice.N(:,2)*lemma;
mN(:,:,3)=lattice.N(:,3)*lemma;

for j=1:(vsize-1)
    
    lr1(:,:,1)=(lattice.VORTEX(:,j,1)*lemma)';
    lr1(:,:,2)=(lattice.VORTEX(:,j,2)*lemma)';
    lr1(:,:,3)=(lattice.VORTEX(:,j,3)*lemma)';
    
    lr2(:,:,1)=(lattice.VORTEX(:,j+1,1)*lemma)';
    lr2(:,:,2)=(lattice.VORTEX(:,j+1,2)*lemma)';
    lr2(:,:,3)=(lattice.VORTEX(:,j+1,3)*lemma)'; 
    
    r1=lr1-mCOLLOC;
    r2=lr2-mCOLLOC;
    
    warning off
    LDW(:,:,j,:)=mega(r1,r2);   % funzione fondamentale
    warning on
end
LDW(find((isnan(LDW(:,:,:,:)))))=0; % per eventuali cancellazioni

DW=-squeeze(sum(LDW,3))*one_by_four_pi; % vettori velocità indotte

dw=sum(DW.*mN,3); % prodotti scalari tra velocità indotta e versori normali
end


function[DW2]=mega(r1,r2)

% è la subroutine per la parte 'geometrica', calcola i FAC e quindi le DW, mancano solo gamma e 1/4pi

%% First part
F1=cross(r1,r2,3);  % ogni vettore ha le componenti x,y,z nelle tre matrici parallele

LF1=(sum(F1.^2,3));



F2(:,:,1)=F1(:,:,1)./(LF1);
F2(:,:,2)=F1(:,:,2)./(LF1);
F2(:,:,3)=F1(:,:,3)./(LF1);
%clear('F1')


%% Next part

Lr1=sqrt(sum(r1.^2,3)); 
Lr2=sqrt(sum(r2.^2,3));


R1(:,:,1)=r1(:,:,1)./Lr1;
R1(:,:,2)=r1(:,:,2)./Lr1;
R1(:,:,3)=r1(:,:,3)./Lr1;

R2(:,:,1)=r2(:,:,1)./Lr2;
R2(:,:,2)=r2(:,:,2)./Lr2;
R2(:,:,3)=r2(:,:,3)./Lr2;



L1=(R2-R1);
%clear('R1','R2')



%% Third part
R0=(r2-r1);

radial_distance=sqrt((LF1./(sum(R0.^2,3))));


%% combinging 2 and 3
L2=  R0(:,:,1).*L1(:,:,1)...
    +R0(:,:,2).*L1(:,:,2)...
    +R0(:,:,3).*L1(:,:,3);



%% Downwash
DW(:,:,1)=F2(:,:,1).*L2;
DW(:,:,2)=F2(:,:,2).*L2;
DW(:,:,3)=F2(:,:,3).*L2;

near=config('near');  % vettore vuoto

DW2(:,:,1)=DW(:,:,1).*(1-(radial_distance<near));
DW2(:,:,2)=DW(:,:,2).*(1-(radial_distance<near));
DW2(:,:,3)=DW(:,:,3).*(1-(radial_distance<near));


end
