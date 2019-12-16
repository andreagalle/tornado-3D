%grafico
clear
%INPUT:
c  = .269;
rp = .076;
raggio  = 0.051;
semilatus = c/2	;		    ;
ecce      = semilatus/rp-1  ;         %eccentricity
semi_corda = c/2/sqrt(1-ecce^2)*sqrt(1-((raggio+ecce*c/2/(1-ecce^2))/(c/2/(1-ecce^2)))^2);

%%%%%%%%%

ypsi = linspace(-semi_corda,semi_corda,1001);
asci = -(semilatus/(1-ecce^2)*sqrt(1-(ypsi/(semilatus/sqrt(1-ecce^2))).^2)-ecce*semilatus/(1-ecce^2));  %ellisse, riferimento centrato sul fuoco
camber = (asci-raggio)/2;
asci_int=ones(1,1001)*raggio;

sym=zeros(1,1001);
y=linspace(-semilatus,semilatus,1001);
ascit=-(semilatus/(1-ecce^2)*sqrt(1-(y/(semilatus/sqrt(1-ecce^2))).^2)-ecce*semilatus/(1-ecce^2));
%%%%%%%%%%%

figure(100)
hold on
xlim([-.2,.2])
ylim([-.15,.15])
plot(ypsi,asci,'k',ypsi,-asci,'k',ypsi,asci_int,'k',ypsi,-asci_int,'k','linewidth',3)

plot(y,sym,'b-.','linewidth',2)
plot(y,ascit,'k:',y,-ascit,'k:')

plot(ypsi,camber,'r',ypsi,-camber,'r','linewidth',3)



axis equal






