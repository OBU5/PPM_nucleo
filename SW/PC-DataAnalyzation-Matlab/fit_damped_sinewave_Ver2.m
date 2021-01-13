% algorythm that was designed by Detlef Amberg (see below)
% https://www.mathworks.com/matlabcentral/fileexchange/50756-fit-a-damped-sine-wave
clear
% this is your signal
load data_part.mat
% make it even number of samples
sig=sig(1:end-1);

%% ========================================================================
% A - Compute Analytic signal from measured signal
% =========================================================================
xx=zeros(1,10);
sigiq=zeros(size(sig));
a0=2/(pi*(0+1));
a2=2/(pi*(2+1));
a4=2/(pi*(4+1));
n=length(sig);

for(k=1:n)
    %calc anlytic signal, 10th order FIR
    xx(1) = sig(k);
    sigiq(k)= xx(5)+ ...
        j*((xx(1)-xx(10))*a0+(xx(2)-xx(8))*a2+(xx(4)-xx(6))*a4);
    xx=[sig(k) xx(1:9)];
end;

% analytic signal with FFT
% freq = 2085.9483Hz
if(0)
    spsig=fft(sig);
    spsig((n/2+1):end)=0;
    sigiq=ifft(spsig);
end;
plot(1:n,real(sigiq),'b.-',1:n,imag(sigiq),'r.-');

% phase run for all of sigiq
%dang=unwrap(angle(sigiq));

%% ========================================================================
% B - convert analytic signal into angles
% =========================================================================
% better: sum up difference phase for any sample
% determine angle between each sample
dang=angle(sigiq(1:end-1)./sigiq(2:end));
% sum the angles to create 
dang=cumsum(dang);

% throw away filter transient
dang=dang(30:end);

%angle difference
diffdang=max(dang)-min(dang);
n=length(dang);

%% ========================================================================
% C - Determine frequency
% =========================================================================
% time needed for one cycle
tau=2*pi*(n/100000)/diffdang;

% 2085.9351Hz, inverse of tau
ff = 1/tau

% adjusted for 0 phase difference at the beginning by hand
% adjusted for 0 phase difference at the end by design
% in between the accumulated phase jitter of the sampled signal
% imho this is a systematic error
sigr1=max(abs(sig)*0.5)*cos(2*pi*ff*(-18+(0:n-1))/100000);


figure
subplot(2,1,1)
plot(1:n,sig(1:n),'r.-',1:n,sigr1,'b.-');
title(['\fontsize{15}Original signal vs reconstructed signal']);



% variable to determine length of zoom on the x axis
zoomLevel = 1000;

% Graph bottom left
subplot(2,4,5)
plot(1:n,sig(1:n),'r.-',1:n,sigr1,'b.-');
xlim([0 zoomLevel])
title('\fontsize{15}beginning of the signal');

% graph bottom middle-left
subplot(2,4,6)
plot(1:n,sig(1:n),'b.-',1:n,sigr1,'r.-');
xlim([length(sig)/3 length(sig)/3 + zoomLevel])
title('\fontsize{15}1/3 of the signal');

% graph bottom middle-right
subplot(2,4,7)
plot(1:n,sig(1:n),'r.-',1:n,sigr1,'b.-');
xlim([length(sig)/3 *2 length(sig)/3 *2 + zoomLevel])
title('\fontsize{15}2/3 of the signal');

% Graph bottom right
subplot(2,4,8)
plot(1:n,sig(1:n),'r.-',1:n,sigr1,'b.-');
xlim([length(sig) - zoomLevel  length(sig)])
title('\fontsize{15}ending of the signal');return