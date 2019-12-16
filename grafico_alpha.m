%ALPHA SWEEP plot

clear

%INPUT LOAD
cd output
load test1-Cx_alpha.mat
aux(:,1,1) = squeeze(results.matrix(1,1,:)); % CL
aux(:,1,2) = squeeze(results.matrix(2,1,:)); % CD
load test2-Cx_alpha.mat
aux(:,2,1) = squeeze(results.matrix(1,1,:));
aux(:,2,2) = squeeze(results.matrix(2,1,:));
load test22-Cx_alpha.mat
aux(:,3,1) = squeeze(results.matrix(1,1,:));
aux(:,3,2) = squeeze(results.matrix(2,1,:));
load test3-Cx_alpha.mat
aux(:,4,1) = squeeze(results.matrix(1,1,:));
aux(:,4,2) = squeeze(results.matrix(2,1,:));
load test32-Cx_alpha.mat
aux(:,5,1) = squeeze(results.matrix(1,1,:));
aux(:,5,2) = squeeze(results.matrix(2,1,:));
load test4-Cx_alpha.mat
aux(:,6,1) = squeeze(results.matrix(1,1,:));
aux(:,6,2) = squeeze(results.matrix(2,1,:));
load test42-Cx_alpha.mat
aux(:,7,1) = squeeze(results.matrix(1,1,:));
aux(:,7,2) = squeeze(results.matrix(2,1,:));

alpha = [0:10]'; 

cd ..

titles  =['A','B','C','D','E','F','G'];
colours =['k','k','k','b','b','r','-.r'];
aux(1,:,2)=0;
%CL
figure(333)
hold on
plot(alpha,aux(:,1,1),'k','linewidth',2.5)
plot(alpha,aux(:,2,1),'k','linewidth',2.5)
plot(alpha,aux(:,3,1),'k--','linewidth',2.5)
plot(alpha,aux(:,4,1),'b','linewidth',2.5)
plot(alpha,aux(:,5,1),'b--','linewidth',2.5)
plot(alpha,aux(:,6,1),'r','linewidth',2.5)
plot(alpha,aux(:,7,1),'r--','linewidth',2.5)
%for i=1:7
    
%    a=colours(i);
 %   plot(alpha,aux(:,i,1),a,'linewidth',2.5)
     
%end
%legend('A','B','C','D','E','F','G','location','Northwest')
ylabel('CL')
xlabel('\alpha')

%CD
figure(334)
hold on
plot(alpha,aux(:,1,2),'k','linewidth',2.5)
plot(alpha,aux(:,2,2),'k','linewidth',2.5)
plot(alpha,aux(:,3,2),'k--','linewidth',2.5)
plot(alpha,aux(:,4,2),'b','linewidth',2.5)
plot(alpha,aux(:,5,2),'b--','linewidth',2.5)
plot(alpha,aux(:,6,2),'r','linewidth',2.5)
plot(alpha,aux(:,7,2),'r--','linewidth',2.5)
% for    
%     a=colours(j);
%     plot(alpha,aux(:,j,2),a,'linewidth',2.5)
%      
% end
%legend('A','B','C','D','E','F','G','location','Northwest' )
ylabel('CD')
xlabel('\alpha')
ylim([0 12*10^-3])
% plot(alpha,aux(:,i,2),'color',a,'linewidth',2.5)
    
%DRAG polar

figure(335)
hold on
plot(aux(:,1,2),aux(:,1,1),'k','linewidth',2.5)
plot(aux(:,1,2),aux(:,2,1),'k','linewidth',2.5)
plot(aux(:,1,2),aux(:,3,1),'k--','linewidth',2.5)
plot(aux(:,1,2),aux(:,4,1),'b','linewidth',2.5)
plot(aux(:,1,2),aux(:,5,1),'b--','linewidth',2.5)
plot(aux(:,1,2),aux(:,6,1),'r','linewidth',2.5)
plot(aux(:,1,2),aux(:,7,1),'r--','linewidth',2.5)
ylabel('CL')
xlabel('CD')



