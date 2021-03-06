function [out]=fStripforce(geo,results,lattice,state,ref,vCfraction)
%This lemma function computes the aerodynamic force on each strip.

q=0.5*state.rho*state.AS^2;
S=ref.S_ref;

F=results.F;                            %Reassigning to save some space
%% Vortex points
[s1 s2 s3]=size(lattice.VORTEX);
if s2==8
    pV1=squeeze(lattice.VORTEX(:,4,:));
    pV2=squeeze(lattice.VORTEX(:,5,:));
elseif s2==4
    pV1=squeeze(lattice.VORTEX(:,2,:));
    pV2=squeeze(lattice.VORTEX(:,3,:));
end
pV=(pV1+pV2)/2;
%%    

[ai bi]=size(geo.nx);                       %number of wings and panels
cnx=geo.nx+geo.fnx;                     %corrected number of xpanels


for i=1:geo.nwing;
    cny(i,:)=geo.ny(i,:).*(geo.symetric(i)+1); %corrected number of ypanels
end

stripsperwing=sum(cny);

%% Compute force action point and  strip moment axis

m=0;

lastindex1=0;
lastindex2=0;
index1=    1+lastindex1;
index2=    cnx(1,1)+lastindex2;

for i=1:ai          %loop per wing
    for j=1:bi      %loop per partition
        for k=1:cny(i,j)  
            %per strip loop
            m=m+1;
            
            %% Compute force action point and  strip moment axis
            cornerp=squeeze([lattice.XYZ(index1,1,:);
                              lattice.XYZ(index1,2,:);
                              lattice.XYZ(index2,3,:);
                              lattice.XYZ(index2,4,:)]);
            
           localC1(m,:)=[(cornerp(1,:)+cornerp(2,:))/2];
           localC2(m,:)=[(cornerp(3,:)+cornerp(4,:))/2];
           Mpoint=(1-vCfraction)*localC1(m,:)+(vCfraction)*localC2(m,:);
           yprimestation(m)=sign(Mpoint(2))*sqrt(Mpoint(2)^2+Mpoint(3)^2);
           
           %Local chord
           lemma1=localC1(m)-localC2(m);
           lc(m)=sqrt(sum(lemma1.^2));
           
           %local span
           lemma1=(-cornerp(1,:)+cornerp(2,:));
           lemma2=lemma1.*[0 1 1];%Disregarding x component
           ls(m)=sqrt(sum(lemma2.^2));
           
           %Strip Area
           la(m)=ls(m)*lc(m);
           
            %%
            %Forces
            F0(m)=sum(sqrt(F(index1:index2,2).^2+F(index1:index2,3).^2)); %Only Z and Y component  
                   
            %Moments
            difference(:,1)=Mpoint(1)-pV(index1:index2,1);
            difference(:,2)=Mpoint(2)-pV(index1:index2,2);
            difference(:,3)=Mpoint(3)-pV(index1:index2,3); 
            F2=F(index1:index2,:);
            M=cross(F2,difference);
            M2=sum(M);
            M3(m)=sqrt(sum(M2.^2));
            
            %% Coefficients
            CZprime(m)=F0(m)/(q*la(m));
            Cmprime(m)=M3(m)/(q*la(m)*lc(m));
  
            index1=index1+cnx(i,j);
            index2=index2+cnx(i,j);
            
            
        end
    end
    [yps or]=sort(yprimestation);
    
    out.ypstation=yps;
    out.stripforce=F0(or);
    out.pitchmoment=M3(or);
    out.CZprime=CZprime(or);
    out.Cmprime=Cmprime(or);
    out.forcepermeter=F0(or)./ls(or);
    out.pitchmomentpermeter=M3(or)./ls(or);
    
    
    
    
    %% Shearload, bending moment and integrated twist moment computation
    stripforce_p=out.stripforce(1:(stripsperwing/2));
    stripforce_sb=out.stripforce((stripsperwing/2+1):end);
    
    load=sum(F0);
    
    shear_p=cumsum(stripforce_p);
    shear_sb=-(fliplr(cumsum(fliplr(stripforce_sb))));
    shear=[shear_p shear_sb];
    
    striptwist_p=out.pitchmoment(1:(stripsperwing/2));
    striptwist_sb=out.pitchmoment((stripsperwing/2+1):end);
    
    twist_p=vumsum
    
    
end
end %function stripforce
        
        
        
        