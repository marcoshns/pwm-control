function [a0,an,bn,freq] = Function2Fourier(x,dt,step)
%% [a0,an,bn,freq] = Function2Fourier(x,dt,step)
% Transforms a function into its Fourier decomposition for a given harmonic
% step. For example if the step is equal to 3, we obtain all the
% coefficients of the harmonics of rank 3. Warning! it is necessary that
% x must be a multiple of the desired frequency
%  a0: mean value of x
%  an: Real part of Fourier decomposition
%  bn: Imaginary part of Fourier decomposition
%  freq: frequencies associated with an and bn without the mean component
%  x: periodic signal to decompose
%  dt: time interval between two values of x

L = max(size(x));
L_demi = floor(L/2);

Fs = 1/dt;
freq = Fs*(0:L_demi)/L;
freq = freq(step+1:step:end);

P2 = fft(x)/L;

cn = P2(1:L_demi+1);
cn(2:end-1) = 2*cn(2:end-1);

a0 = cn(1);
an = real(cn(step+1:step:end));
bn = -imag(cn(step+1:step:end));


end