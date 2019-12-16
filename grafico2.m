%grafico2

clear

%INPUT:
ci  = [.269 , .269, .269 , .269, .269, .289, .3];
rpi = [.076, .076, .076, .086, .103, .076, .076];
raggioi  = [0.051, .04, .032, .061, .073, .051, .051];
titles =['A','B','C','D','E','F','G'];
figure(02)
for i=2:7    
subplot (3,2,i-1)
%set(gca,'visible','off')

hold on
%INPUT:
c  = ci(i) ;
rp = rpi(i);
raggio  = raggioi(i);
semilatus = c/2	;		    ;
ecce      = semilatus/rp-1  ;         %eccentricity
semi_corda = c/2/sqrt(1-ecce^2)*sqrt(1-((raggio+ecce*c/2/(1-ecce^2))/(c/2/(1-ecce^2)))^2);

%%%%%%%%%

ypsi = linspace(-semi_corda,semi_corda,10001);
asci = -(semilatus/(1-ecce^2)*sqrt(1-(ypsi/(semilatus/sqrt(1-ecce^2))).^2)-ecce*semilatus/(1-ecce^2));  %ellisse, riferimento centrato sul fuoco
camber = (asci-raggio)/2;
asci_int=ones(1,10001)*raggio;

sym=zeros(1,10001);
y=linspace(-semilatus,semilatus,10001);
ascit=-(semilatus/(1-ecce^2)*sqrt(1-(y/(semilatus/sqrt(1-ecce^2))).^2)-ecce*semilatus/(1-ecce^2));
%%%%%%%%%%%

%xlim([-.2,.2])
%ylim([-.15,.15])
%plot(ypsi,asci,'k',ypsi,-asci,'k',ypsi,asci_int,'k',ypsi,-asci_int,'k','linewidth',3)

plot(y,sym,'b-.','linewidth',2)
%plot(y,ascit,'k:',y,-ascit,'k:')

plot(ypsi,camber,'r',ypsi,-camber,'r','linewidth',3)

title(titles(i),'fontsize',20)

axis equal
axis off
end


