clc, clearvars, close all

%% Lab04: Generation, Processing, and Analysis of Sine Waves

%% 1) Read Data
% Load the provided signal 'signal.csv' using the 'csvread()' function and
% split the signal and time into two separate vectors.

fprintf('1) Start.\n')



M = readmatrix("signal.csv");
%disp (M);
t= M(:,4); %time vector from column 4
volt=M(:,5); %voltage vector from column 5


% Plot the signal
figure(1)
plot(t,volt);

fprintf('1) Done.\n')


%% 2/3) Butterworth Filter
% Design a Butterworth filter that fulfills the required characteristics 
% given in the assignment description. 
% Use the built-in functions of
% Matlab. The 'doc' and 'help' functions might be useful to obtain detailed
% information.

% 2) First, calculate the required filter order and cutoff frequency and
% print the result.

fprintf('2) Start.\n')


Hw_s=(1/(sqrt(1+(15/30)^2)))^3;
display (Hw_s);

fprintf('Hw_s > 0.7, is good!\n')

Hw_n=(1/(sqrt(1+(200/30)^2)))^3;
display (Hw_n);
fprintf('Hw_n < 0.01, is good!\n')

fprintf('2) Done.\n')

% 3) Calculate the filter coefficients and apply them to the signal, i.e.,
% filter the signal. Plot the filtered signal into the same figure as the
% original signal. Make sure to add a title, the axis descriptions, and a
% legend.

fprintf('3) Start.\n')

fc =30;
fs = 5000;
fny =fs/2;
fprintf('Nyquist Freq:\n')
disp(fny);
wn = fc/fny;
fprintf('wn:\n')
disp(wn);
[b,a]=butter(3,wn);
z=filter(b,a,volt);
figure(2);
plot(z);

fprintf('3) Done.\n')

%% 4. Fourier Transform
% Calculate the single-sided Fourier transform of the filtered signal.

fprintf('4) Start.\n')

% 4.1) First, obtain the length of the original and filtered signal and 
% calculate their means. Print both mean values.
fprintf('Original Signal lenght:\n')
[s ,q] = size (volt)

fprintf('Original Mean value:\n')
mean_o=sum(volt)/s

fprintf('Filtred Signal lenght:\n')
[s_f ,q_f] = size (z)

fprintf('Filtred Mean value:\n')
mean_f=sum(z)/s_f
% 4.2) Do the FFT for both signals; see the docs and lecture slides for
% help. Make sure to remove the mean from the signals.

% 4.2.1.) Original signal

f1=fft(volt,s);

% 4.2.2) Filtered signal

f2=fft(z,s_f);


% 4.2.3) When plotting, only visualize the spectrum of to 500 Hz.


P2o = abs(f1/s);
P1o = P2o(1:s/2+1);
P1o(1)=P1o(1)-mean_o;
P1o(2:end-1) = 2*P1o(2:end-1);
f = fs*(0:(s/2))/s;
figure(3)
plot(f,P1o) 
xlim([0 500])


P2f = abs(f2/s);
P1f = P2f(1:s/2+1);
P1f(1)=P1f(1)-mean_f;
P1f(2:end-1) = 2*P1f(2:end-1);
f = fs*(0:(s/2))/s;
figure(4)
plot(f,P1f) 
xlim([0 500])


fprintf('4) Done.\n')

%% 5. Frequency Identification
% Write a function that automatically detects a signals frequency based
% on its frequency spectrum. You can assume there's only a single signal
% and noise has been removed. The function must return the amplitude and
% the frequency of this signal.

fprintf('5) Start.\n')


% 5.2) What is the frequency of they signal you have analyzed?
freq_o=f(find(max(P1o)==P1o))
freq_f=f(find(max(P1f)==P1f))



fprintf('5) Done.\n')

% 5.1) Define function

function har=hrm(data) 
    abs(real(data)).^2 
end


 