function y=schem(p,q)
%% this function generate the geometric objects for plot

%y=[phi1;phi2;dis1;dis2;len;b;dis0;rho0;rho1;rho2;k1;k2;k3;c1;c2;c3];
e1=[1;0]; e2=[0;1]; 

phi1=p(1); dis1=p(3); rho1=p(10:11,:); 
phi2=p(2); dis2=p(4); rho2=p(12:13,:);

Cp1=cos(phi1); Sp1=sin(phi1);
Cp2=cos(phi2); Sp2=sin(phi2);

%% directions of the rails 
u1=[Cp1;Sp1]; v1=[-Sp1;Cp1]; 
u2=[Cp2;Sp2]; v2=[-Sp2;Cp2]; 

q1=q(1); q2=q(2); q3=q(3);

%% rails
rG1=-dis1*v1; rH1=rG1+0.3*u1; Rail1=[rG1,rH1];
rG2=-dis2*v2; rH2=rG2+0.3*u2; Rail2=[rG2,rH2];

% plot(Rail1(1,:),Rail1(2,:),'b','linewidth',2);
% hold on;
% plot(Rail2(1,:),Rail2(2,:),'b','linewidth',2);

%% XY stage
 bp0=p(6);
rho0=p(8:9,:); 
rE0=[q1;q2];
rE1=rE0+e1*2.5e-3;
rE2=rE0-e1*2.5e-3;
rF1=rE0-rho0+bp0*e2*Sp1+e1*2.5e-3;
rF2=rE0-rho0+bp0*e2*Sp1-e1*2.5e-3;
rA1=rE0-rho0+rho1-bp0*u1;
rB1=rE0-rho0+rho1+bp0*u1;
rB2=rE0-rho0+rho2+bp0*u2;
rA2=rE0-rho0+rho2-bp0*u2;

plat=[rE1,rF1,rB1,rA1,rA2,rB2,rF2,rE2];
% fill(plat(1,:),plat(2,:),'b','Facealpha',0.5);

%% end-effector
phi0=q3;
Cp0=cos(phi0); Sp0=sin(phi0);
u0=[Cp0;Sp0]; v0=[-Sp0;Cp0];

dis0=p(7);
rA0=rE0+dis0*v0;

rad0=0.23/2;
the0=0:pi/30:pi*2;
obj_x=rad0*cos(the0)+rA0(1);
obj_y=rad0*sin(the0)+rA0(2);
obj=[obj_x;obj_y];

%% object
% fill(obj(1,:),obj(2,:),'b','Facealpha',0.3);

%gripper
alp0=12*pi/180; dphi=pi/6;
alp1=phi0-pi/2+alp0: dphi/20:phi0-pi/2+alp0+dphi;
alp2=phi0-pi/2-alp0:-dphi/20:phi0-pi/2-alp0-dphi;

grip1=[rad0*cos(alp1)+rA0(1);rad0*sin(alp1)+rA0(2)];
grip2=[rad0*cos(alp2)+rA0(1);rad0*sin(alp2)+rA0(2)];

rH2=grip2(:,1)+(rad0*cos(alp0)-dis0)*v0/2;
rH1=grip1(:,1)+(rad0*cos(alp0)-dis0)*v0/2;
rH0=(rH1+rH2)/2;

grip=[fliplr(grip2),rH2,rH0,rE0,rH0,rH1,grip1];
% plot(grip(1,:),grip(2,:),'color',[0.25,0.6,0.25],'linewidth',3);

phi0=0:pi/30:2*pi;
Rj1=[0.005*cos(phi0)+rE0(1);0.005*sin(phi0)+rE0(2)];
% fill(Rj1(1,:),Rj1(2,:),'r','Facealpha',1);

%% links
% CM of stage
len=p(5);
rp0=rE0-rho0;
pds1=dis1+v1'*(rp0+rho1); the1=asin(pds1/len);
pds2=dis2+v2'*(rp0+rho2); the2=asin(pds2/len);

% directions of links
Ct1=cos(the1); St1=sin(the1); w1=u1*Ct1+v1*St1;
Ct2=cos(the2); St2=sin(the2); w2=u2*Ct2+v2*St2;

rC1=rA1-len*w1; rD1=rB1-len*w1;
rC2=rA2-len*w2; rD2=rB2-len*w2;

link11=[rA1,rC1]; link12=[rB1,rD1]; 
link21=[rA2,rC2]; link22=[rB2,rD2]; 

%% sliders
rC11=rC1-u1*7.5e-3+v1*7.5e-3; rC12=rC1-u1*7.5e-3-v1*7.5e-3;
rD11=rD1+u1*7.5e-3+v1*7.5e-3; rD12=rD1+u1*7.5e-3-v1*7.5e-3;

rC21=rC2-u2*7.5e-3+v2*7.5e-3; rC22=rC2-u2*7.5e-3-v2*7.5e-3;
rD21=rD2+u2*7.5e-3+v2*7.5e-3; rD22=rD2+u2*7.5e-3-v2*7.5e-3;

slid1=[rC11,rC12,rD12,rD11];
slid2=[rC21,rC22,rD22,rD21];
% fill(slid1(1,:),slid1(2,:),'r','Facealpha',1);
% fill(slid2(1,:),slid2(2,:),'r','Facealpha',1);

% plot(link11(1,:),link11(2,:),'color',[0.25,0.6,0.25],'linewidth',3);
% plot(link12(1,:),link12(2,:),'color',[0.25,0.6,0.25],'linewidth',3);
% plot(link21(1,:),link21(2,:),'color',[0.25,0.6,0.25],'linewidth',3);
% plot(link22(1,:),link22(2,:),'color',[0.25,0.6,0.25],'linewidth',3);

%% output the geometry
y={obj,grip,plat,Rj1,slid1,slid2,...
  link11,link12,link21,link22,Rail1,Rail2};