# Pulse Width Modulation (PWM) Control: Application to Electric Vehicle (EV)/Hybrid Electric Vehicles

The DC/AC converter (i.e., inverter) transforms a continuous signal into an alternate signal. This conversion process is extremely important for applications where the goal is to control the frequency and amplitude of the output signal such as Photovoltaic (PV) systems connected to the grid or propulsion control of an electric motor present in an Electric Vehicle (EV). The figure below presents a 3-phase inverter supplied by a continuous voltage provided by a battery pack and feeding an inductive load such as a synchronous motor.

![](images/3-phase inverter.PNG)

The control of the inverter's commutation cells is done through a boolean signal (0 - open switch or 1 - closed switch) called Pulse-Width Modulation (PWM). In the literature, there are numerous PWM methods available, thus, this work will address the following methods to control the 3-phase inverter: 

 - Sinusoidal Pulse-Width Modulation (SPWM)
 - Third-Harmonic Injection Pulse-Width Modulation $1/6$ (THIPWM $1/6$)
 - Space Vector Pulse-Width Modulation (SVPWM)
 - Space Vector Modulation (SVM)
 - Discontinuous Pulse-Width Modulation Minimal (DPWMMin)
 - Near-State Pulse-Width Modulation (NSPWM)
 
In order to simulate and obtain the results of the Fourier analysis, WTHD and THD, please run the MATLAB file "pwm_fourier_analysis.m". The Simulink file "pwm_control.slx" is presented. File "Function2Fourier.m" contains the function to compute the Fourier analysis. 

MATLAB/Simulink Version 2022a
