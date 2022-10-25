% E-PiCo - Control of Power Converters for Electric Propulsion System (COPCEV)
% École Centrale de Nantes - ECN
%
% Author: Marcos Henrique do Nascimento Sousa
% e-mail: marcos.sousa@eleves.ec-nantes.fr
%
% Reference: Bourgeade Adrien, 2022. "DC-AC conversion, Application to EV/HEV
% electrical motor", École Centrale de Nantes, p. 96, September 2022.
%

close all
close all
clc

%---------------------- DC-AC System Parameters ---------------------------
fs = 1000;              % Switching frequency [Hz]
fm = 50;                % Modulation frequency [Hz] 
Edc = 400;              % DC-Bus voltage [V] 
Vr = 180;               % Reference voltage [V]
theta_ref = 0;          % Reference angle [°]
dt = 1e-5;              % Simulation step
m = Vr/Edc;             % Modulation gain
m_nspwm = 0.45;         % Modulation gain NSPWM


%------------------------ Load parameters ---------------------------------
L = 1e-3;               % Inductance [H]
R = 0.01;                  % Resistence [Ohm]

%------------------------- Simulink file ----------------------------------

out = sim('pwm_control',0.1);

%--------------------------- Fourier analysis -----------------------------
step = 1;               % Harmonic step

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SPWM Control %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[a0_spwm,an_spwm,bn_spwm,freq_spwm] = Function2Fourier(out.v1n_spwm,dt,step);
H_spwm = sqrt(an_spwm.^2+ bn_spwm.^2);

% Plotting
f1 = figure(1);
set(f1,'name','Fourier analysis SPWM Control: Triangular and Sawtooth Carrier');
plot(freq_spwm(5:5:250),H_spwm(5:5:250));
grid on                 % Add grid
[tx1] = title('Fourier analysis SPWM Control: Triangular and Sawtooth Carrier for $$m=0.45$$, $$f1 = 50 Hz$$','Interpreter','latex');
tx1.FontSize = 12; set(tx1, 'Interpreter', 'latex');
xlabel('f [Hz]', 'Interpreter', 'latex');
ylabel('Voltage amplitude', 'Interpreter', 'latex');
ax = gca; ax.FontSize = 12;


%%% WTHD
vn_spwm = H_spwm(5:5:end);
spwm_length= length(vn_spwm); 
n_spwm = (1:spwm_length)';

wthd_spwm = (100/vn_spwm(1,1))*sqrt(sum((vn_spwm(2:end,1).^2)./(n_spwm(2:end,1).^2)));

%%% THD
thd_spwm = 100*sqrt(sum(vn_spwm(2:end,1).^2)./sum(vn_spwm.^2));

%%%%%%%%%%%%%%%%%%%%%%%%%%% THIPWM 1/6 Control %%%%%%%%%%%%%%%%%%%%%%%%%%%%
[a0_thipwm,an_thipwm,bn_thipwm,freq_thipwm] = Function2Fourier(out.v1n_thipwm,dt,step);
H_thipwm = sqrt(an_thipwm.^2+ bn_thipwm.^2);

% Plotting
f2 = figure(2);
set(f2,'name','Fourier analysis THIPWM 1/6 Control');
plot(freq_thipwm(5:5:500),H_thipwm(5:5:500));
grid on                 % Add grid
[tx2] = title('Fourier analysis THIPWM $$1/6$$ Control for $$m=0.45$$, $$f1 = 50 Hz$$','Interpreter','latex');
tx2.FontSize = 12; set(tx2, 'Interpreter', 'latex');
xlabel('f [Hz]', 'Interpreter', 'latex');
ylabel('Voltage amplitude', 'Interpreter', 'latex');
ax = gca; ax.FontSize = 12;


%%% WTHD
vn_thipwm = H_thipwm(5:5:end);
thipwm_length= length(vn_thipwm); 
n_thipwm = (1:thipwm_length)';

wthd_thipwm = (100/vn_thipwm(1,1))*sqrt(sum((vn_thipwm(2:end,1).^2)./(n_thipwm(2:end,1).^2)));

%%% THD
thd_thipwm = 100*sqrt(sum(vn_thipwm(2:end,1).^2)./sum(vn_thipwm.^2));

%%%%%%%%%%%%%%%%%%%%%%%%%%% SVPWM Control %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[a0_svpwm,an_svpwm,bn_svpwm,freq_svpwm] = Function2Fourier(out.v1n_svpwm,dt,step);
H_svpwm = sqrt(an_svpwm.^2+ bn_svpwm.^2);

% Plotting
f3 = figure(3);
set(f3,'name','Fourier analysis SVPWM Control');
plot(freq_svpwm(5:5:500),H_svpwm(5:5:500));
grid on                 % Add grid
[tx3] = title('Fourier analysis SVPWM Control for $$m=0.45$$, $$f1 = 50 Hz$$','Interpreter','latex');
tx3.FontSize = 12; set(tx3, 'Interpreter', 'latex');
xlabel('f [Hz]', 'Interpreter', 'latex');
ylabel('Voltage amplitude', 'Interpreter', 'latex');
ax = gca; ax.FontSize = 12;


%%% WTHD
vn_svpwm = H_svpwm(5:5:end);
svpwm_length= length(vn_svpwm); 
n_svpwm = (1:svpwm_length)';

wthd_svpwm = (100/vn_svpwm(1,1))*sqrt(sum((vn_svpwm(2:end,1).^2)./(n_svpwm(2:end,1).^2)));

%%% THD
thd_svpwm = 100*sqrt(sum(vn_svpwm(2:end,1).^2)./sum(vn_svpwm.^2));


%%%%%%%%%%%%%%%%%%%% SPWM Control (Sawthooth Carrier) %%%%%%%%%%%%%%%%%%%%%
[a0_spwm_sawtooth,an_spwm_sawtooth,bn_spwm_sawtooth,freq_spwm_sawtooth] = Function2Fourier(out.v1n_spwm_sawtooth,dt,step);
H_spwm_sawtooth = sqrt(an_spwm_sawtooth.^2+ bn_spwm_sawtooth.^2);

% Plotting
figure(1);
%set(f1,'name','Fourier analysis SPWM strategy control');
hold on
plot(freq_spwm_sawtooth(5:5:250),H_spwm_sawtooth(5:5:250));
[lx1] = legend('SPWM - Triangular Carrier', 'SPWM - Sawtooth Carrier', 'Interpreter', 'latex');
%grid on                 % Add grid
%[tx1] = title('Fourier analysis SPWM strategy control for $$m=0.45$$','Interpreter','latex');
%tx1.FontSize = 12; set(tx1, 'Interpreter', 'latex');
%xlabel('f [Hz]', 'Interpreter', 'latex');
%ylabel('Voltage amplitude', 'Interpreter', 'latex');
%ax = gca; ax.FontSize = 12;


%%% WTHD
vn_spwm_sawtooth = H_spwm_sawtooth(5:5:end);
spwm_length_sawtooth= length(vn_spwm_sawtooth); 
n_spwm_sawtooth = (1:spwm_length_sawtooth)';

wthd_spwm_sawtooth = (100/vn_spwm_sawtooth(1,1))*sqrt(sum((vn_spwm_sawtooth(2:end,1).^2)./(n_spwm_sawtooth(2:end,1).^2)));

%%% THD
thd_spwm_sawtooth = 100*sqrt(sum(vn_spwm_sawtooth(2:end,1).^2)./sum(vn_spwm_sawtooth.^2));

%%%%%%%%%%%%%%%%%%%%%%%%%%% SVM Control %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[a0_svm,an_svm,bn_svm,freq_svm] = Function2Fourier(out.v1n_svm,dt,step);
H_svm = sqrt(an_svm.^2+ bn_svm.^2);

% Plotting
f4 = figure(4);
set(f4,'name','Fourier analysis SVM Control');
plot(freq_svm(5:5:500),H_svm(5:5:500));
grid on                 % Add grid
[tx4] = title('Fourier analysis SVM Control for $$m=0.45$$, $$f1 = 50 Hz$$','Interpreter','latex');
tx4.FontSize = 12; set(tx4, 'Interpreter', 'latex');
xlabel('f [Hz]', 'Interpreter', 'latex');
ylabel('Voltage amplitude', 'Interpreter', 'latex');
ax = gca; ax.FontSize = 12;


%%% WTHD
vn_svm = H_svm(5:5:end);
svm_length= length(vn_svm); 
n_svm = (1:svm_length)';

wthd_svm = (100/vn_svm(1,1))*sqrt(sum((vn_svm(2:end,1).^2)./(n_svm(2:end,1).^2)));

%%% THD
thd_svm = 100*sqrt(sum(vn_svm(2:end,1).^2)./sum(vn_svm.^2));

%%%%%%%%%%%%%%%%%%%%%%%%%%% DPWM Control %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[a0_dpwm,an_dpwm,bn_dpwm,freq_dpwm] = Function2Fourier(out.v1n_dpwm,dt,step);
H_dpwm = sqrt(an_dpwm.^2+ bn_dpwm.^2);

% Plotting
f5 = figure(5);
set(f5,'name','Fourier analysis DPWM Control');
plot(freq_dpwm(5:5:500),H_dpwm(5:5:500));
grid on                 % Add grid
[tx5] = title('Fourier analysis DPWM Control for $$m=0.45$$, $$f1 = 50 Hz$$','Interpreter','latex');
tx5.FontSize = 12; set(tx5, 'Interpreter', 'latex');
xlabel('f [Hz]', 'Interpreter', 'latex');
ylabel('Voltage amplitude', 'Interpreter', 'latex');
ax = gca; ax.FontSize = 12;


%%% WTHD
vn_dpwm = H_dpwm(5:5:end);
dpwm_length= length(vn_dpwm); 
n_dpwm = (1:dpwm_length)';

wthd_dpwm = (100/vn_dpwm(1,1))*sqrt(sum((vn_dpwm(2:end,1).^2)./(n_dpwm(2:end,1).^2)));

%%% THD
thd_dpwm = 100*sqrt(sum(vn_dpwm(2:end,1).^2)./sum(vn_dpwm.^2));

%%%%%%%%%%%%%%%%%%%%%%%%%%% NSPWM Control %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[a0_nspwm,an_nspwm,bn_nspwm,freq_nspwm] = Function2Fourier(out.v1n_nspwm,dt,step);
H_nspwm = sqrt(an_nspwm.^2+ bn_nspwm.^2);

% Plotting
f6 = figure(6);
set(f6,'name','Fourier analysis NSPWM Control');
plot(freq_nspwm(5:5:500),H_nspwm(5:5:500));
grid on                 % Add grid
[tx6] = title('Fourier analysis NSPWM Control for $$m=0.45$$, $$f1 = 50 Hz$$','Interpreter','latex');
tx6.FontSize = 12; set(tx6, 'Interpreter', 'latex');
xlabel('f [Hz]', 'Interpreter', 'latex');
ylabel('Voltage amplitude', 'Interpreter', 'latex');
ax = gca; ax.FontSize = 12;


%%% WTHD
vn_nspwm = H_nspwm(5:5:end);
nspwm_length= length(vn_nspwm); 
n_nspwm = (1:nspwm_length)';

wthd_nspwm = (100/vn_nspwm(1,1))*sqrt(sum((vn_nspwm(2:end,1).^2)./(n_nspwm(2:end,1).^2)));

%%% THD
thd_nspwm = 100*sqrt(sum(vn_nspwm(2:end,1).^2)./sum(vn_nspwm.^2));

%%%%%%%%%%%%%%%%%%%%%%%% Load Current Analysis %%%%%%5%%%%%%%%%%%%%%%%%%%%%
f7 = figure(7);
set(f7,'name','Load Current');
plot(out.tout,out.iload_spwm);
hold on
plot(out.tout,out.iload_thipwm);
hold on
plot(out.tout,out.iload_svpwm);
hold on
plot(out.tout,out.iload_dpwm);
hold on
plot(out.tout,out.iload_svm);
hold on 
plot(out.tout,out.iload_nspwm);
[lx7] = legend('SPWM', 'THIPWM $$1/6$$', 'SVPWM', 'DPWM', 'SVM', 'NSPWM', 'Interpreter', 'latex');
grid on                 % Add grid
[tx7] = title('Current analysis for an inductive load $$L=0.001 \ [H]$$','Interpreter','latex');
tx7.FontSize = 12; set(tx7, 'Interpreter', 'latex');
xlabel('t [s]', 'Interpreter', 'latex');
ylabel('Current [A]', 'Interpreter', 'latex');
ax = gca; ax.FontSize = 12;

%%%%%%%%%%%%%%%%%%%%%%%% DPWM Duty Cycle %%%%%%%%%%%%%%%%%%%%%%%%%%%
f8 = figure(8);
set(f8,'name','DPWM Switching Signal');
subplot(211), plot(out.tout,out.dpwm_mod_carrier);
[lx81] = legend('Modulation Signal', 'Carrier Sginal', 'Interpreter', 'latex');
grid on
[tx81] = title('Modulation and carrier signal','Interpreter','latex');
tx81.FontSize = 12; set(tx81, 'Interpreter', 'latex');
xlabel('t [s]', 'Interpreter', 'latex');
ylabel('Amplitude', 'Interpreter', 'latex');
ax = gca; ax.FontSize = 12;

hold on
subplot(212), plot(out.tout,out.dpwm_s1);
grid on
[tx82] = title('S1','Interpreter','latex');
tx82.FontSize = 12; set(tx82, 'Interpreter', 'latex');
xlabel('t [s]', 'Interpreter', 'latex');
ylabel('Amplitude', 'Interpreter', 'latex');
ax = gca; ax.FontSize = 12;

%%%%%%%%%%%%%%%%%%%%%%%% K SVM %%%%%%%%%%%%%%%%%%%%%%%%%%%
f9 = figure(9);
set(f9,'name','SVM k sequence signal');
plot(out.tout,out.k_svm);
%[lx9] = legend('Modulation Signal', 'Carrier Sginal', 'Interpreter', 'latex');
grid on
[tx9] = title('SVM k-sequence signal','Interpreter','latex');
tx9.FontSize = 12; set(tx9, 'Interpreter', 'latex');
xlabel('t [s]', 'Interpreter', 'latex');
ylabel('Amplitude', 'Interpreter', 'latex');
ax = gca; ax.FontSize = 12;

%%%%%%%%%%%%%%%%%%%%%%%% K NSPWM %%%%%%%%%%%%%%%%%%%%%%%%%%%
f10 = figure(10);
set(f10,'name','NSPWM k sequence signal');
plot(out.tout,out.k_nspwm);
%[lx9] = legend('Modulation Signal', 'Carrier Sginal', 'Interpreter', 'latex');
grid on
[tx10] = title('NSPWM k-sequence signal','Interpreter','latex');
tx10.FontSize = 12; set(tx10, 'Interpreter', 'latex');
xlabel('t [s]', 'Interpreter', 'latex');
ylabel('Amplitude', 'Interpreter', 'latex');
ax = gca; ax.FontSize = 12;

%%%%%%%%%%%%%%%%%%%%%%%% NSPWM switching signal %%%%%%%%%%%%%%%%%%%%%%%%%%%
f11 = figure(11);
set(f11,'name','NSPWM Switching Signal S1, S2, S3');
subplot(311), plot(out.tout,out.s1_nspwm);
%[lx9] = legend('Modulation Signal', 'Carrier Sginal', 'Interpreter', 'latex');
grid on
[tx111] = title('S1','Interpreter','latex');
tx111.FontSize = 12; set(tx111, 'Interpreter', 'latex');
xlabel('t [s]', 'Interpreter', 'latex');
ylabel('Amplitude', 'Interpreter', 'latex');
ax = gca; ax.FontSize = 12;

hold on
subplot(312), plot(out.tout,out.s2_nspwm);
%[lx9] = legend('Modulation Signal', 'Carrier Sginal', 'Interpreter', 'latex');
grid on
[tx112] = title('S2','Interpreter','latex');
tx112.FontSize = 12; set(tx112, 'Interpreter', 'latex');
xlabel('t [s]', 'Interpreter', 'latex');
ylabel('Amplitude', 'Interpreter', 'latex');
ax = gca; ax.FontSize = 12;

hold on
subplot(313), plot(out.tout,out.s3_nspwm);
%[lx9] = legend('Modulation Signal', 'Carrier Sginal', 'Interpreter', 'latex');
grid on
[tx113] = title('S3','Interpreter','latex');
tx113.FontSize = 12; set(tx113, 'Interpreter', 'latex');
xlabel('t [s]', 'Interpreter', 'latex');
ylabel('Amplitude', 'Interpreter', 'latex');
ax = gca; ax.FontSize = 12;

%hold on 
%subplot(313), plot(out.tout,out.dpwm_s3);
%[tx83] = title('S3','Interpreter','latex');
%tx83.FontSize = 12; set(tx83, 'Interpreter', 'latex');
%xlabel('t [s]', 'Interpreter', 'latex');
%ylabel('Amplitude', 'Interpreter', 'latex');
%ax = gca; ax.FontSize = 12;

%[lx7] = legend('SPWM', 'THIPWM $$1/6$$', 'SVPWM', 'DPWM', 'Interpreter', 'latex');
%grid on                 % Add grid
%[tx8] = title('S1','Interpreter','latex');
%tx8.FontSize = 12; set(tx8, 'Interpreter', 'latex');
%xlabel('t [s]', 'Interpreter', 'latex');
%ylabel('Amplitude', 'Interpreter', 'latex');
%ax = gca; ax.FontSize = 12;

%------------------------ Printing Results --------------------------------
fprintf('Harmonic Distortion Analysis:\n');
fprintf('PWM                          WTHD     THD         Fundamental Amplitude\n');
fprintf('SPWM - Triangular Carrier    %2.4f   %2.4f     %2.4f\n',[wthd_spwm,thd_spwm,vn_spwm(1,1)]);
fprintf('SPWM - Sawtooth Carrier      %2.4f   %2.4f     %2.4f\n',[wthd_spwm_sawtooth,thd_spwm_sawtooth,vn_spwm_sawtooth(1,1)]);
fprintf('THIPWM                       %2.4f   %2.4f     %2.4f\n',[wthd_thipwm,thd_thipwm,vn_thipwm(1,1)]);
fprintf('SVPWM                        %2.4f   %2.4f     %2.4f\n',[wthd_svpwm,thd_svpwm,vn_svpwm(1,1)]);
fprintf('SVM                          %2.4f   %2.4f     %2.4f\n',[wthd_svm,thd_svm,vn_svm(1,1)]);
fprintf('DPWM                         %2.4f   %2.4f     %2.4f\n',[wthd_dpwm,thd_dpwm,vn_dpwm(1,1)]);
fprintf('NSPWM                        %2.4f   %2.4f     %2.4f\n',[wthd_nspwm,thd_nspwm,vn_nspwm(1,1)]);