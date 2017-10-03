function y=parameters(x)

%% preset the kinematics of the mechanism

%% kinematics of rails: direction + position
phi1=60*pi/180;
phi2=pi-phi1;

dis1= 0.075; 
dis2=-0.075;

%% link and slider length
len=0.200;
  b=0.025;

%% kineatics of gripper
dis0=0.125;
  
%% platform dimension
rho0=[    0;0.025];
rho1=[ 0.025;    0];
rho2=[-0.025;    0];

%% joint stiffness + damping
k1=250;   c1=2.0;
k2=250;   c2=2.0;
k3=0.125*1; c3=0.020*1;

y=[phi1;phi2;dis1;dis2;len;b;dis0;rho0;rho1;rho2;k1;k2;k3;c1;c2;c3];

