clear all
%% System parameters
s_m  = 5333;        % Sprung Mass (kg)
us_m = 906.5;       % Unsprung Mass (kg)
k_s  = 430000;      % Spring Constant (N/m)
k_us = 2440000;     % Wheel stiffness (N/m)
b_s  = 20000;       % Suspension Damping coefficient (sec/m)
b_us = 40000;       % Wheel Damping coefficient (sec/m)
%% Steady Space Representation
A = [ 0 1 0 -1 ;
   -k_s/s_m -b_s/s_m 0 b_s/s_m;
     0 0 0 1;
   k_s/us_m b_s/us_m -k_us/us_m -(b_s+b_us)/us_m];
B = [0  0 ;
    0 1/s_m ;
   -1  0 ;
   b_us/us_m -1/us_m ];
C = [ 1 0 0 0 ;
   -k_s/s_m -b_s/s_m 0 b_s/s_m ];
D = [0 0;
    0 0;
    0 0;
    0 0;
    0 0;
    0 1/s_m];
%% Controllability
rank(ctrb(A,B));
%% LQR Control law
Q = diag([1760*10^6, 11.6*10^6, 1, 1]);
R = 0.01;
K = lqr( A, B(:,2), Q, R );
%% Setting of Simulation Time
Simulation_Time= 10;
sim('Suspension_model')
%% Plotting Rail Body Acceleration
x1 = Acceleration_Act.time;
y1 = Acceleration_Act.data;
y2 = Acceleration_Pas.data;
figure (1)
p=plot(x1,y1,'r',x1,y2,'b')
p(2).LineStyle  = '--';
grid
title ('Time response of Sprung mass acceleration')
xlabel('Time (in seconds)')
ylabel('Acceleration (in m/s2)');
legend({'Active','Passive'},'FontSize',16,'FontWeight','bold')
print('-f1','Time response of the Sprung mass acceleration','-dpng','-r0')
%% Plotting Suspension Travel Distance
x2 = Suspension_Travel_Act.time;
y3 = Suspension_Travel_Act.data;
y4 = Suspension_Travel_Pas.data;
figure (2)
p=plot(x2,y3,'r',x2,y4,'b')
p(2).LineStyle  = '--';
grid
title ('Time response of the Suspension Travel')
xlabel('Time (in seconds)')
ylabel('Travel (in m)');
legend({'Active','Passive'})
print('-f2','Time response of the Suspension Travel','-dpng','-r0')
%% Plotting Wheel Deflection
x3 = Deflection_Act.time;
y5 = Deflection_Act.data;
y6 = Deflection_Pas.data;
figure (3)
p=plot(x3,y5,'r',x3,y6,'b')
p(2).LineStyle  = '--';
grid
title ('Time response of the Wheel Deflection')
xlabel('Time (secs)')
ylabel('Wheel Deflection (m)');
legend({'Active','Passive'})
print('-f3','Time response of the Wheel Deflection','-dpng','-r0')
%% Plots of Body Displacements
% For Passive System
x4 = Displacement_Pas.time;
y7 = Displacement_Pas.data;
figure (4)
p=plot(x4,y7)
grid
title ('Time response of the Body Displacements (Passive System)')
xlabel('Time (in seconds)')
ylabel('Body Displacement (in m)');
legend({'Sprung mass','Track profile','Unsprung mass'})
print('-f4','Time response of the Body Displacements (Passive System)','-dpng','-r0')
% For Active System
x5 = Displacement_Act.time;
y8 = Displacement_Act.data;
figure (5)
p=plot(x5,y8)
grid
title ('Time response of the Body Displacements (Active System)')
xlabel('Time (in seconds)')
ylabel('Displacement (in m)'); 
legend({'Sprung mass','Track profile','Unsprung mass'})
print('-f5','Time response of the Body Displacements (Active System)','-dpng','-r0')
%% Plot of the Actuator Force
x6 = Actuator_Force.time;
y9 = Actuator_Force.data;
figure (6)
p=plot(x6,y9)
grid
title ('Time response of Actuator Force')
xlabel('Time (in second)')
ylabel('Force (in Newton)');
legend({'Actuator force'})
print('-f6','Time response of the force generated from the actuator','-dpng','-r0')

