% Code that was wrote by Detlef Amberg (see below)
% https://www.mathworks.com/matlabcentral/fileexchange/50756-fit-a-damped-sine-wave
clear
load data_part.mat
sig=sig(1:end-1);
spsig=fft(sig);
n=length(sig);
spsig(n/2+1:end)=0;
sigh=ifft(spsig);
ww=unwrap(angle(sigh));
ww=ww(80:end);
ww=ww(1:end-50);
% Phase and frequency jitter. very bad
phasejitter=diff(ww);
ff=mean(phasejitter);
sigart=abs(sigh).'.*exp((0:n-1)*j*ff); % artificial, no jitter
sigartr=real(sigart);
wws=ww-linspace(ww(1),ww(end),length(ww)).';
(2*pi)/(100000/2085);

fit_damped_sinewave(sigartr);

return