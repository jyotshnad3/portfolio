%% Magnetic Levitation Control Design

%% Model Parameters
m = 20e-3;                    % Mass, kg
g = 9.81;                       % Gravitational constant, m/s^2
alpha = 1.241578125e-3*m;       % Magnetic force constant, N*m^2/(A^2)

gamma = 143.48;                 % Sensor gain, V/m
s0 = 0.022;                     % Sensor offset, m

umin = -5;                      % Minimum input, A
umax = 5;                       % Maximum input, A

%% Equilbrium Point
% FILL IN YOUR WORK BELOW
% Equilibrium height, current, and output.

% Equilibrium height and current
hbar = [0.009];
ibar = [0.79998];

% Equilbrium output
ybar = [-1.86524];

% Set initial conditions for simulation
h0 = hbar;
hdot0 = 0;

% Simulate to final time Tf
Tf = 0.5;
sim('maglevOpenLoop',Tf);

% Plot Results: Measurement in V
figure(1);
plot(y.Time,y.Data);
grid on;
ylabel('Output y, V');

%% Linearization
% FILL IN YOUR WORK BELOW
N=-2*alpha/m *ibar/(hbar^2)
D=( -2*alpha/(m*(hbar^3))*ibar^2)
H= tf(N,[1 0 D])
G=gamma*H
dcgain(G)

%% PID controller design
% FILL IN YOUR WORK BELOW
Ts=3; 
%Ts=3/(zeta*wn)
zeta=1;
wn=6.3/3;
p=6.3/3;
kd=(p+(2*zeta*wn)) / -3518.125;
kp=((2*zeta*wn*p+wn^2) +2180) / -3518.125;
 ki=wn^2*p/-3518.125;
 %% 
% kp=-0.6234 *1
% ki=-0.0026 *10
% kd=-0.0018 *11.5
% 
% kp=-1
% ki=-0.0026  *10
% kd=-0.0018 *11.5
%%GainMargin: 0.6199 GMFrequency: 1.1207 PhaseMargin: 49.8088
% 
% 
% kp=-0.7
% ki=-0.0026  *25
% kd=-0.0018 *11.5
% p =
% 
%   -68.7679
%    -2.9386
%    -1.1318

% kp=-0.8
% ki=-0.0026  *25
% kd=-0.0018 *11.5
% C=tf([kd kp ki],[1 0])
% T=minreal(G*C/(1+G*C));
% % figure(10)
% % step(T)
% % pole(T)

% kp=-0.7
% ki=-0.0026  *27
% kd=-0.0018 *10  %kd decrease phase margin decreases
% kp=-1.2505
% ki=-0.6165
% kd=-0.019825
kp=-2.15
ki=-1.6
kd=-0.029
C=tf([kd kp ki],[1 0])
T=minreal(G*C/(1+G*C));
L=C*G;
allmargin(L)
p = pole(T)
stepinfo(T,"settlingtimeThreshold",0.05)
p = esort(p);
% Time constants of all poles
tau = 1./abs(real(p));
% The dominant (slowest) poles are a complex pair.
p1 = p(1);
p2 = p(2);
p3=p(3);


%% Pre-filter to shape reference
% FILL IN YOUR WORK BELOW
Fs=tf([ki],[kd kp ki])
PreComp=minreal(Fs*T);
step(PreComp)
stepinfo(PreComp,"settlingtimeThreshold",0.05)


%% Stability Margins
% FILL IN YOUR WORK BELOW


%% Nonlinear Simulation
% FILL IN YOUR WORK BELOW: Add your controller to Simulink diagram and 
% generate plots for various reference commands.

% Set Simulation Parameters
NoiseVariance = 5e-5;          % Sensor noise variance, m^2
StepValue     = 0.004*gamma;    % Magnitude of step, V  
StepTime      =    5;           % Time when step input occurs, sec
h0 = s0;                        % Initial ball position, m
hdot0 = 0;                      % Initial ball velocity, m/s

% Select reference signal: Step Input
%  RefSignal=1 for  step input
%  RefSignal=2 for  squarewave input
%  RefSignal=3 for  staircase input
RefSignal = 2;          % Index used by Reference Selector block

% Simulate
Tf = 60;
sim('maglevClosedLoop',Tf);

figure(12)
plot(y);