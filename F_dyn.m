function [yout,For_R]=F_dyn(qin,par)
%% this function generate the acceleration of generalized coordinates
%% according to the current state
% clear; clc;

%% input state: configuration and velocity
q=qin(1:3,:);
v=qin(4:6,:);

%% variables of state 
pox=q(1); poy=q(2); the=q(3);
vq1=v(1); vq2=v(2); vq3=v(3);


I2=eye(2); Del=[0, 1; -1, 0];
I23=[1,0,0;0,1,0];
%directions of rails
phi1=par(1); Cp1=cos(phi1); Sp1=sin(phi1);
phi2=par(2); Cp2=cos(phi2); Sp2=sin(phi2);

u1=[Cp1;Sp1]; v1=[-Sp1;Cp1];
u2=[Cp2;Sp2]; v2=[-Sp2;Cp2];

%% distances of the rails to the origin
dis1_0=par(3); dis2_0=par(4);

%% joint stiffness and damping
k1=par(14); k2=par(15); k3=par(16);
c1=par(17); c2=par(18); c3=par(19);

%% body-joint vectors
rho0=par( 8: 9,:);
rho1=par(10:11,:);
rho2=par(12:13,:);

  b=par(6);

%% length of the link
len=par(5);

%% XY stage: position of center of mass
r_p=[pox;poy]-rho0;

%% kinematics analysis
dis1=dis1_0+v1'*(r_p+rho1); the1=asin(dis1/len); 
dis2=dis2_0+v2'*(r_p+rho2); the2=asin(dis2/len); 

%% unit vectors of the links
Ct1=cos(the1); St1=sin(the1); w1=Ct1*u1+St1*v1;
Ct2=cos(the2); St2=sin(the2); w2=Ct2*u2+St2*v2;

%% import the home position of end-effector 
rp00 = evalin('base','rp0');

%% the corresponding home positions of prismatic joints
p10=u1'*(rp00-rho0+rho1)-(len^2-(dis1_0+v1'*(rp00-rho0+rho1))^2)^0.5;
p20=u2'*(rp00-rho0+rho2)-(len^2-(dis2_0+v2'*(rp00-rho0+rho2))^2)^0.5;

%% current positions of the prismatic joints.
p1=u1'*(r_p+rho1-w1*len);
p2=u2'*(r_p+rho2-w2*len);

%% the Jacobian of the links' directions
J_the1=[v1'/(len*Ct1),0]; v_the1=J_the1*v;
J_the2=[v2'/(len*Ct2),0]; v_the2=J_the2*v;

%% orientation of end-effector
Cth=cos(the); Sth=sin(the); 
u0=[Cth;Sth]; v0=[-Sth;Cth];

%% distance of the object's center of mass to the revolute joint
%% namely the body-joint vector of object
dis0=par(7); 

%% derive the full Jacobian and reaction force co-efficients of all bodies
%% target object
Jac0=[1, 0, -dis0*Cth;...
      0, 1, -dis0*Sth;...
      0, 0,        1];
  
alp0=[dis0*Sth*vq3^2;...
     -dis0*Cth*vq3^2;...
                  0];

Q0=zeros(3,22);
Q0(1:2,1:2)=I2;
Q0(3,1:2)=-dis0*v0'*Del;

Fb0=[0;0;-k3*the-c3*vq3];

%% XY stage
Jac1=[1, 0, 0;...
      0, 1, 0;...
      0, 0, 0];
alp1=[0;0;0];

Q1=zeros(3,22);
Q1(1:2,1:10)=[-I2,I2,I2,I2,I2];
Q1(3,1:10)=[-rho0'*Del,...
            (rho1-b*u1)'*Del,...
            (rho1+b*u1)'*Del,...
            (rho2-b*u2)'*Del,...
            (rho2+b*u2)'*Del,...
            ];

Fb1=[0;0;k3*the+c3*vq3];        
%% link 11
Jac2=[I23-0.5*len*(-u1*St1+v1*Ct1)*J_the1;...
      J_the1];
alp2=[0.5*len*w1*v_the1^2;0];   

Q2=zeros(3,22);
Q2(1:2,[3,4,11,12])=[-I2,I2];
Q2(3,[3,4,11,12])=[-0.5*len*w1'*Del,-0.5*len*w1'*Del];

Fb2=[0;0;0];  
%% link12
Jac3=[I23-0.5*len*(-u1*St1+v1*Ct1)*J_the1;...
      J_the1];
alp3=[0.5*len*(u1*Ct1+v1*St1)*v_the1^2;0];   

Q3=zeros(3,22);
Q3(1:2,[5,6,13,14])=[-I2,I2];
Q3(3,[5,6,13,14])=[-0.5*len*w1'*Del,-0.5*len*w1'*Del];

Fb3=[0;0;0]; 
%% link21
Jac4=[I23-0.5*len*(-u2*St2+v2*Ct2)*J_the2;...
      J_the2];
alp4=[0.5*len*(u2*Ct2+v2*St2)*v_the2^2;0];

Q4=zeros(3,22);
Q4(1:2,[7,8,15,16])=[-I2,I2];
Q4(3,[7,8,15,16])=[-0.5*len*w2'*Del,-0.5*len*w2'*Del];

Fb4=[0;0;0]; 
%% link22
Jac5=[I23-0.5*len*(-u2*St2+v2*Ct2)*J_the2;...
      J_the2];
alp5=[0.5*len*(u2*Ct2+v2*St2)*v_the2^2;0];

Q5=zeros(3,22);
Q5(1:2,[9,10,17,18])=[-I2,I2];
Q5(3,[9,10,17,18])=[-0.5*len*w2'*Del,-0.5*len*w2'*Del];

Fb5=[0;0;0]; 
%% slider 1
Jac6=[I23-len*(-u1*St1+v1*Ct1)*J_the1;...
      0,0,0];
alp6=[len*(u1*Ct1+v1*St1)*v_the1^2;0];

Q6=zeros(3,22);
Q6(1:2,[11,12,13,14,19,20])=[-I2,-I2,v1*[1,0]];
Q6(3,[11,12,13,14,19,20])=[b*u1'*Del,-b*u1'*Del,0,1];

Fb6=[(-k1*(p1-p10)-c1*u1'*Jac6(1:2,:)*v)*u1;0];

%% slider 2
Jac7=[I23-len*(-u2*St2+v2*Ct2)*J_the2;...
      0,0,0];
alp7=[len*(u2*Ct2+v2*St2)*v_the2^2;0];

Q7=zeros(3,22);
Q7(1:2,[15,16,17,18,21,22])=[-I2,-I2,v2*[1,0]];
Q7(3,[15,16,17,18,21,22])=[b*u2'*Del,-b*u2'*Del,0,1];

Fb7=[(-k2*(p2-p20)-c2*u2'*Jac7(1:2,:)*v)*u2;0];

%%%%%%%%%%%%%%%%%%%%%%%%%
Qr=[Q0;Q1;Q2;Q3;Q4;Q5;Q6;Q7];

%% System Full Jacobian: 24-by-3
H_Jac=[Jac0;Jac1;Jac2;Jac3;Jac4;Jac5;Jac6;Jac7];

%% extra item of accelerations: 24-by-1
C_alp=[alp0;alp1;alp2;alp3;alp4;alp5;alp6;alp7];

%% extra item of active forces (excluding the reaction ones): 24-by-1
F_bia=[Fb0;Fb1;Fb2;Fb3;Fb4;Fb5;Fb6;Fb7];

%% mass and inertia of links
m0=1.55; J0=0.013; mas0=diag([m0;m0;J0]);
m1=0.0; J1=0.0; mas1=diag([m1;m1;J1]);
m2=0; J2=0; mas2=diag([m2;m2;J2]);
m3=0; J3=0; mas3=diag([m3;m3;J3]);
m4=0; J4=0; mas4=diag([m4;m4;J4]);
m5=0; J5=0; mas5=diag([m5;m5;J5]);
m6=0; J6=0; mas6=diag([m6;m6;J6]);
m7=0; J7=0; mas7=diag([m7;m7;J7]);

%% system inertia matrix 24-by-24
Mas=blkdiag(mas0,mas1,mas2,mas3,mas4,mas5,mas6,mas7);

%% check out if H_Jac and Qr are orthogonal to each other.
%% Jud should always be zero 
% Jud=norm(H_Jac'*Qr);

%% equations of motion
Psy=H_Jac'*Mas*H_Jac;

a=Psy\H_Jac'*(F_bia-Mas*C_alp);

%% equations of reaction
CoA=Qr'*Qr;
CoB=Qr'*(Mas*H_Jac*a+Mas*C_alp-F_bia);

%% reaction forces
For_R=pinv(CoA)*CoB;

%% derivative of generalized coordiantes' state
yout=[v;a];


