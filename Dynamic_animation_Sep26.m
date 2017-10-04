clear; clc;

e1=[1;0]; e2=[0;1];    % unit vectors of coordinate axes
rp0=[0;0.250];         % home position of tip-frame

q0=[rp0;0];           % initial position of tip-frame
v0=[0.25;-0.25; 0];   % initial velocity of tip-frame 
v0=[0; 0.25; 0];   % initial velocity of tip-frame 

y0=[q0;v0];           % initial state
gre=[0.25,0.6,0.25];  % re-define the color green :-)

para=parameters([]);  % load the kinematic parameters

t=2;                  % simulation time
h=1e-2;               % step length
n=t/h;                % number of steps 
x(1)=0;               % initialize time
y(:,1)=y0;            % initialize state

subplot(1,2,1);       % plot the mechanism

%% xy axes
plot([0,0.1],[0,0],'r','linewidth',1.0);
hold on;
plot([0,0],[0,0.1],'b','linewidth',1.0);
axis equal;
axis([-0.35,0.35,-0.05,0.85]);

text(0.1,-0.025,'$x$',...
     'Interpreter','latex','FontSize',14);
text(-0.05,0.1,'$y$',...
     'Interpreter','latex','FontSize',14);

%% generate the geometry of mechanism using the function 'schem.m'
sch0=schem(para,q0);
    obj=sch0{1};   grip=sch0{2};   plat=sch0{3}; 
    Rj1=sch0{4};  slid1=sch0{5};  slid2=sch0{6};  
 link11=sch0{7}; link12=sch0{8}; 
 link21=sch0{9}; link22=sch0{10};
  Rail1=sch0{11}; Rail2=sch0{12};

%% plot the mechanism with different geometric objects
 Obj0=fill(obj(1,:),obj(2,:),'b','Facealpha',0.3);
 Gri0=plot(grip(1,:),grip(2,:),'color',gre,'linewidth',2);
 Pla0=fill(plat(1,:),plat(2,:),'b','Facealpha',0.5);
 Rai1=plot(Rail1(1,:),Rail1(2,:),'b','linewidth',2);
 Rai2=plot(Rail2(1,:),Rail2(2,:),'b','linewidth',2);
 Sli1=fill(slid1(1,:),slid1(2,:),'r','Facealpha',1);
 Sli2=fill(slid2(1,:),slid2(2,:),'r','Facealpha',1);
 Ln11=plot(link11(1,:),link11(2,:),'color',gre,'linewidth',3);
 Ln12=plot(link12(1,:),link12(2,:),'color',gre,'linewidth',3);
 Ln21=plot(link21(1,:),link21(2,:),'color',gre,'linewidth',3);
 Ln22=plot(link22(1,:),link22(2,:),'color',gre,'linewidth',3);
 Rjn0=fill(Rj1(1,:),Rj1(2,:),'r','Facealpha',1);
 
 %% list the stiffness and damping parameters
 k=text(-0.325,0.80,...
     ['$k_1=k_2=$',num2str(para(14)),'$N/m,\;k_{\theta}=$',...
      num2str(para(16)),'$\,Nm$'],...
     'Interpreter','latex','FontSize',12);
 c=text(-0.325,0.74,...
     ['$b_1=b_2=$',num2str(para(17)),'$Ns/m,\;b_{\theta}=$'...
      num2str(para(19)),'$Nms$'],...
     'Interpreter','latex','FontSize',12);
 %% initial state
 Ini=text(-0.325,0.68,...
     ['$v_{0}=[$',num2str(v0'),'$]^{T}$'],...
     'Interpreter','latex','FontSize',12);
 
 %% simulation time
 Timer=text(0.115,0.68,'\;$t=0\,s$',...
     'Interpreter','latex','FontSize',12);
 
 %% plot the force curves
 subplot(1,2,2);
 lnx=plot(0,0,'r');
 hold on; grid on;
 lny=plot(0,0,'color',gre);
 lnz=plot(0,0,'b');
 axis([0,t,-10,10]);
%  legend(lnx,'F_x','interpreter','latex');
 legend([lnx,lny,lnz],'F_u','F_v','M_{\theta}');
 
for ii=1:n
    x(ii+1)=x(ii)+h; % time
    
    %% [yout,For_R]=F_dyn(qin,par) is the dynamics function
    %% input the current state 'qin', 
    %% output its derivative 'yout' and reaction force 'For_R'
    %% ode45 algorithm    
    [k1,Frc1]=F_dyn(y(:,ii),para);
    [k2,Frc2]=F_dyn(y(:,ii)+h*k1/2,para);
    [k3,Frc3]=F_dyn(y(:,ii)+h*k2/2,para);
    [k4,Frc4]=F_dyn(y(:,ii)+h*k3,para);
    y(:,ii+1)=y(:,ii)+h*(k1+2*k2+2*k3+k4)/6;
    
    %% abstract the forces exerted on the object 
    Reac1(:,ii)=Frc1(1:2,:);
    Torq(ii)=-y(3,ii)*para(16)-y(6,ii)*para(19);
    
    %% update the geometry of mechanism according to updated state
    schk=schem(para,y(:,ii+1));
    
    obje=schk{1};  grip=schk{2};  plat=schk{3}; 
    Rjoi=schk{4};  sli1=schk{5};  sli2=schk{6};  
    lk11=schk{7};  lk12=schk{8}; 
    lk21=schk{9};  lk22=schk{10};
    
    %% update the datum of plot
    set(Obj0,'Xdata',obje(1,:),'Ydata',obje(2,:));
    set(Gri0,'Xdata',grip(1,:),'Ydata',grip(2,:));
    set(Pla0,'Xdata',plat(1,:),'Ydata',plat(2,:));
    set(Rjn0,'Xdata',Rjoi(1,:),'Ydata',Rjoi(2,:));
    set(Sli1,'Xdata',sli1(1,:),'Ydata',sli1(2,:));
    set(Sli2,'Xdata',sli2(1,:),'Ydata',sli2(2,:));
    set(Ln11,'Xdata',lk11(1,:),'Ydata',lk11(2,:));
    set(Ln12,'Xdata',lk12(1,:),'Ydata',lk12(2,:));
    set(Ln21,'Xdata',lk21(1,:),'Ydata',lk21(2,:));
    set(Ln22,'Xdata',lk22(1,:),'Ydata',lk22(2,:)); 
    set(Timer,'String',['$t=$',num2str(x(ii+1)),'\,s']);

%     set(lnx,'Xdata',x,'Ydata',y(1,:)*1e3); 
%     set(lny,'Xdata',x,'Ydata',y(2,:)*1e3-300); 
%     set(lnz,'Xdata',x,'Ydata',y(3,:)*180/pi); 
    %% update the force datum
    set(lnx,'Xdata',x(1:ii),'Ydata',Reac1(1,:)); 
    set(lny,'Xdata',x(1:ii),'Ydata',Reac1(2,:)); 
    set(lnz,'Xdata',x(1:ii),'Ydata',Torq); 
    set(gcf,'doublebuffer','on');
    %% pause for a step time
    pause(h);
    drawnow;
end

