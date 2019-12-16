%LOCAL CIRCULATION

clear

%INPUT
aux=[];

cd output
load test1-Cx.mat
aux(:,1) = results.gamma;
%load test2-Cx.mat
%aux(:,2) = results.gamma;
%load test22-Cx.mat
%aux(:,3) = results.gamma;
%load test3-Cx.mat
%aux(:,4) = results.gamma;
%load test32-Cx.mat
%aux(:,5) = results.gamma;
%load test4-Cx.mat
%aux(:,6) = results.gamma;
%load test42-Cx.mat
%aux(:,7) = results.gamma;
cd ..

titles  =['A','B','C','D','E','F','G'];
colours =['b','g','k','c','r','y','m'];

figure(222)
hold on
for i=1:1
    a=colours(i);
    appo = aux(:,i);
    %LEADING EDGE
    %appo = appo(1:42:1764);
    appo1 = appo(1:84:end);
    appo2 = appo(end-41:-84:43);
    length(appo1);
    length(appo2);
    appo=[appo1 ;appo2];
    %figure(8000)
    %plot([1:21],appo1,'g')
    plot([1:42],appo,a,'linewidth',3)
    ylim([-0.02 0.025])
    xlim([1,42])
    grid on
    
    %figure(800)
    %  colormap(hot)
    %   fill3(lattice.XYZ(43:84:end/2,:,1)',lattice.XYZ(43:84:end/2,:,2)',lattice.XYZ(43:84:end/2,:,3)',results.gamma(43:84:end/2)')
    % axis equal
    
   %TRAILING EDGE
    appo = aux(:,i);
    
    appo1 = appo(42:84:end);
    appo2 = appo(end:-84:43);
    %length(appo1)
    %length(appo2)
    appo=[appo1 ;appo2];
    figure(8001)
    %hold on
    %plot([1:21],appo1,'g')
    %xlim([1,42])
    plot([1:42],appo,a,'linewidth',3)
    ylim([-8.5*10^-4 -7.8*10^-4])
    xlim([1,42])
    grid on
    %grid on
    % figure(1)
    %figure(800)
    %ylim([-.00085 -.00075])
   
end
%legend('A')
  