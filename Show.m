function Show()
%SHOWNET Summary of this function goes here
%   Detailed explanation goes here
global Options;
global Vehicles;
clf
figure('Color','w','Position',[100 100 700 600])
set(gca,'FontSize',8,'YGrid','off')
xlabel('\it x \rm [m] \rightarrow')
ylabel('\it y \rm [m] \rightarrow')
hold on;

for i = 1:Options.NumVehicles
    plot(Vehicles(i).Position(1,1),Vehicles(i).Position(1,2),'ko','MarkerSize',10,'MarkerFaceColor','r','color','r');
    text(Vehicles(i).Position(1,1)+1,Vehicles(i).Position(1,2)+1,int2str(i),'FontSize',12,'VerticalAlignment','Baseline','color','k');
    hold on;
end
axis([0 Options.Width+(Options.Width/10) 0 Options.Heigh+(Options.Heigh/10)]);
 for i=1:Options.NumVehicles
     for j=i:Options.NumVehicles
         if ((j~=i) && Vehicles(i).Neighbours(j) == 1)
             X1=Vehicles(i).Position(1,1);
             Y1=Vehicles(i).Position(1,2);
             X2=Vehicles(j).Position(1,1);
             Y2=Vehicles(j).Position(1,2);
             vertice1=[X1,X2];
             vertice2=[Y1,Y2];
             plot(vertice1,vertice2,'-.b','LineWidth',0.1);
             hold on;
             pause(0.005);
         end
         
     end
 end
drawnow;
end

