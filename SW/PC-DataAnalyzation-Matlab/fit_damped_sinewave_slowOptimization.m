% Original code that was made by Detlef Amberg can be found on web page
% https://www.mathworks.com/matlabcentral/fileexchange/50756-fit-a-damped-sine-wave
% function [sigR parameters sigRO parametersO]=fit_damped_sinewave_my(sig)

%This function is divided into 4 sections
%
% A - Initialization
%       1 - Determination of samples per time period of wave
%       2 - Decimation
%
% B - Find the unknown constants "b" and "2*sqrt(b)*cos(w)"
%
% C - Calculation of parameters A, alpha, w and phase from the difference equation
%
% D - Optimalization of the found parameters

%% ========================================================================
% A - Initialization
% =========================================================================
%initialization is divided into 2 subsections
% 1 - Determination of samples per time period of wave
% 2 - Decimation

tic
% set true to display the progress of this function, otherwise set false
displayOn = true;

%load signal data
load('data_part.mat')
% -------------------------------------------------------------------------
% 1 - Determination of samples per time period of wave
% -------------------------------------------------------------------------
sig=sig(:);                         % elements in column order
spd=fft(sig);                       % get dft (discrete fouriere transformation) of signal
[y,ind]=max(abs(spd(1:end/2)));     % do a coarse frequency estimation
ind = ind - 1;                      % ind = number of sinewaves in window
toc

samplesPerPeriod=length(sig) / ind; % samples per 1 period of sine wave
% -------------------------------------------------------------------------



% -------------------------------------------------------------------------
% 2 - Decimation
% -------------------------------------------------------------------------
% In order to achieve sampling frequncy close to nyquist/2 withou loss
% of information 3 functions shifted by "delay = samplesPerPeriod / 4" are
% created. With this feature the signals are shifted PI/4 and that is
% the cos function steepest

delay=round(samplesPerPeriod/4);
n=length(sig);

% linear fit to the difference equation
% x(k+2) = 2*sqrt(b)*cos(w)*x(k+1)-b*x(k)
shiftedSig0=sig(2*delay+1:end);             % vector of x(k+2)
shiftedSig1=sig(1*delay+1:end-delay);       % vector of x(k+1)
shiftedSig2=sig(1:end-2*delay);             % vector of x(k)
% -------------------------------------------------------------------------
if(displayOn)
    disp('A - Initialization')
    fprintf('\tsamples per period = %f\n',samplesPerPeriod);
    fprintf('\tsignals are shifted by %d samples\n',delay);
    fprintf('\n');
    
end

% =========================================================================



%% ========================================================================
% B - Find the unknown constants "b" and "2*sqrt(b)*cos(w)"
% =========================================================================

M1=[-shiftedSig1 -shiftedSig2];
coeff1=inv(M1'*M1)*M1'*shiftedSig0;
% calculate poles of the discrete system x^2 +coeff(1)x + coeff(2)
rest = roots([1; coeff1]);
% correction because of decimation
rest1 = rest.^(1/delay);
far=poly(rest1);    % returns the coefficients of the polynomial
if(displayOn)
    disp('B - Find the unknown constants "b" and "2*sqrt(b)*cos(w)" - DONE ')
    fprintf('\tFound constants are:\n');
    fprintf('\tb                = %f\n',far(3));
    fprintf('\t2*sqrt(b)*cos(w) = %f\n',far(2));
    fprintf('\n');
end
% =========================================================================



%% ========================================================================
% Optional - Reconstruction of the signal from difference equation
%            using the constants found in subsection 3
% =========================================================================

% change to 1 to run the optional part
if(0)
    % find general solution from particular solutions
    % using initial values,
    % optional, omit, if no reconstructed signals from
    % difference equ. are needed
    
    % repmat(rest1.',n,1)   - Create array with n copies of rest1
    % repmat((0:n-1).',1,2) - Create array of 2 columns from 0 to n-1
    % M2 is then array of complex exponential function
    
    M2 = repmat(rest1.',n,1).^repmat((0:n-1).',1,2);
    coeff2=inv(M2.'*M2)*M2.'*sig;
    % sig3r is the reconstructed signal from the difference equation
    sig3r=real(M2*coeff2);
    
    
    % calculate fitted signal from difference equation
    sig1r=zeros(1,n);
    sig1r(1)=sig3r(1);
    sig1r(2)=sig3r(2);
    for(k=3:n)
        sig1r(k)=-far(2)*sig1r(k-1)-far(3)*sig1r(k-2);
    end
end
% =========================================================================



%% ========================================================================
% C - Calculation of parameters A, alpha, w and phase from the difference equation
% =========================================================================

% x(k)= Ar*exp(alphar*k) * cos(wr*k+phr)
% from difference equ.
% x(k+2) = 2*sqrt(b)*cos(w)*x(k+1)-b*x(k)

%b
%b = 3rd coef from polynom far
b=far(3);

%alpha
%exp(alpha)=sqrt(b)
alpha=log(sqrt(b));

%w
%-far(2)= 2*sqrt(b)*cos(w)
w=acos(-far(2)/2/sqrt(b));

%A and phase
M1=[ (exp(alpha*(0:n-1)).*sin((0:n-1)*w)).' ....
    (exp(alpha*(0:n-1)).*cos((0:n-1)*w)).'];
coeff3 = inv(M1'*M1)*M1'*sig;

complexNumber  = coeff3(2)+j*coeff3(1);

%A is represented as magnitude of compleNumber
A  = abs(complexNumber);

%phase is represented as atan2(imag(h), real(h))
phase = angle(complexNumber');



% calculate fitted signal from explicit equ.
sigR=A*exp(alpha*(0:n-1)) .* cos(w*(0:n-1)+phase);
parameters = [A alpha w phase];

if(displayOn)
    disp('C - calculatd A, alpha, w and phase')
    fprintf('\tFound parameters are:\n');
    fprintf('\tA     = %.10f\n',parameters(1));
    fprintf('\talpha = %.10f\n',parameters(2));
    fprintf('\tw     = %.10f\n',parameters(3));
    fprintf('\tphase = %.10f\n',parameters(4));
    fprintf('\n');
end
% =========================================================================




%% ========================================================================
% D - Optimalization of the found parameters
% =========================================================================
% change to 0 to disable the optimalization part
if(1)
    % init
    % The index increases each cycle by 1. The loop is terminated when
    % maxIndex is reached
    index = 1;
    maxIndex = 10;
    
    
    %----------------------------------------------------------------------
    % init alpha optimizer
    %----------------------------------------------------------------------
    alphaOptimizerOn = true;
    stepsOfAlphaOptimizer = 1000;
    minOfAlphaOptimizer = 0.2;
    maxOfAlphaOptimizer = 5;
    alphaOptimizer=minOfAlphaOptimizer:...
        (maxOfAlphaOptimizer-minOfAlphaOptimizer)/stepsOfAlphaOptimizer:...
        maxOfAlphaOptimizer;
    %----------------------------------------------------------------------

    
    
    %----------------------------------------------------------------------
    % init omega optimizer
    %----------------------------------------------------------------------
    omegaOptimizerOn = true;
    stepsOfOmegaOptimizer = 1000;
    minOfOmegaOptimizer = 0.99;
    maxOfOmegaOptimizer = 1.01;
    omegaOptimizer=minOfOmegaOptimizer:...
        (maxOfOmegaOptimizer-minOfOmegaOptimizer)/stepsOfOmegaOptimizer:...
        maxOfOmegaOptimizer;
    %----------------------------------------------------------------------
    
    
    
    %----------------------------------------------------------------------
    % init phase optimizer
    %----------------------------------------------------------------------
    phaseOptimizerOn = true;
    stepsOfPhaseOptimizer = 1000;
    minOfPhaseOptimizer = 0.99;
    maxOfPhaseOptimizer = 1.01;
    phaseOptimizer=minOfPhaseOptimizer:...
        (maxOfPhaseOptimizer-minOfPhaseOptimizer)/stepsOfPhaseOptimizer:...
        maxOfPhaseOptimizer;
    %----------------------------------------------------------------------
    
    
    
    %----------------------------------------------------------------------
    % init Amplitude optimizer
    %----------------------------------------------------------------------
    amplitudeOptimizerOn = true;
    stepsOfAmplitudeOptimizer = 1000;
    minOfAmplitudeOptimizer = 0.2;
    maxOfAmplitudeOptimizer = 5;
    amplitudeOptimizer=minOfAmplitudeOptimizer:...
        (maxOfAmplitudeOptimizer-minOfAmplitudeOptimizer)/stepsOfAmplitudeOptimizer:...
        maxOfAmplitudeOptimizer;
    %----------------------------------------------------------------------
       
    if(displayOn)
        disp('D - Optimalization of the found parameters')
        fprintf('\talpha optimizer     = %f : %f : %f \n',minOfAlphaOptimizer,...
        (maxOfAlphaOptimizer-minOfAlphaOptimizer)/stepsOfAlphaOptimizer,...
        maxOfAlphaOptimizer);
    
        fprintf('\tomega optimizer     = %f : %f : %f \n',minOfOmegaOptimizer,...
        (maxOfOmegaOptimizer-minOfOmegaOptimizer)/stepsOfOmegaOptimizer,...
        maxOfOmegaOptimizer);
    
        fprintf('\tphase optimizer     = %f : %f : %f \n',minOfPhaseOptimizer,...
        (maxOfPhaseOptimizer-minOfPhaseOptimizer)/stepsOfPhaseOptimizer,...
        maxOfPhaseOptimizer);
    
        fprintf('\tAmplitude optimizer = %f : %f : %f \n',minOfAmplitudeOptimizer,...
        (maxOfAmplitudeOptimizer-minOfAmplitudeOptimizer)/stepsOfAmplitudeOptimizer,...
        maxOfAmplitudeOptimizer);
    end
    
    
    if(displayOn)
        fprintf('\tbeginning of optimizing loop\n')
    end
    while(1)
        
        if(displayOn)
            fprintf('\t%d. cycle of optimization out of %d\n',index,maxIndex)
        end
        % Optimalize alpha
        if(alphaOptimizerOn)
            % create empty array
            evaluationOfAlphaOptimizer=zeros(1,length(alphaOptimizer));
            
            for(k=1:length(alphaOptimizer))
                % generate signal with modified damping constant
                % sig=A*exp(alpha*(0:n-1)).*cos(w*(0:n-1)+ph);
                
                optimizedSignal=A*exp(alphaOptimizer(k)*alpha*(0:n-1)).* cos(w*(0:n-1)+phase);
                
                % create "evaluation" for each modification of alpha
                evaluationOfAlphaOptimizer(k)=sum(abs(sig-optimizedSignal(:)));
            end
            
            % find the best value of alphaOptimizer (min of evaluation function)
            [y,indexAlpha]=min(evaluationOfAlphaOptimizer);
            
            if(displayOn)
                fprintf('\t\talpha optimization\n')
                fprintf('\t\t\talpha optimizer = %f\n', alphaOptimizer(indexAlpha))
                fprintf('\t\t\talpha is changed from %f to %f\n',alpha, alphaOptimizer(indexAlpha)*alpha)
                fprintf('\n')
            end
            %Change alpha to new optimized value
            alpha=alphaOptimizer(indexAlpha)*alpha;
            
        end
        
        
        % Optimalize Omega
        if(omegaOptimizerOn)
            % create empty array
            evaluationOfOmegaOptimizer=zeros(1,length(omegaOptimizer));
            
            for(k=1:length(omegaOptimizer))
                % generate signal with modified damping constant
                % sig=A*exp(omega*(0:n-1)).*cos(w*(0:n-1)+ph);
                
                optimizedSignal=A*exp(alpha*(0:n-1)).* cos((w*omegaOptimizer(k))*(0:n-1)+phase);
                
                % create "evaluation" for each modification of alpha
                evaluationOfOmegaOptimizer(k)=sum(abs(sig-optimizedSignal(:)));
            end
            
            % find the best value of omegaOptimizer (min of evaluation function)
            [y,indexOmega]=min(evaluationOfOmegaOptimizer);
            
            
            if(displayOn)
                fprintf('\t\tomega optimization\n')
                fprintf('\t\t\tomega optimizer = %f\n',omegaOptimizer(indexOmega))
                fprintf('\t\t\tomega is changed from %f to %f\n',w, omegaOptimizer(indexOmega) * w)
                fprintf('\n')
            end
            %Change omega to new optimized value
            w=omegaOptimizer(indexOmega) * w;
        end
        
        
        % Optimalize phase
        if(phaseOptimizerOn)
            % create empty array
            evaluationOfPhaseOptimizer=zeros(1,length(phaseOptimizer));
            
            for(k=1:length(phaseOptimizer))
                % generate signal with modified damping constant
                % sig=A*exp(alpha*(0:n-1)).*cos(w*(0:n-1)+ph);
                
                optimizedSignal=A*exp(alpha*(0:n-1)).* cos(w*(0:n-1)+(phase*phaseOptimizer(k)));

                % create "evaluation" for each modification of phase
                evaluationOfPhaseOptimizer(k)=sum(abs(sig-optimizedSignal(:)));
            end
            
            % find the best value of phaseOptimizer (min of evaluation function)
            [y,indexPhase]=min(evaluationOfPhaseOptimizer);
                            
            if(displayOn)
                fprintf('\t\tphase optimization\n')
                fprintf('\t\t\tphase optimizer = %f\n',phaseOptimizer(indexPhase))
                fprintf('\t\t\tphase is changed from %f to %f\n', phase, phaseOptimizer(indexPhase)*phase)
                fprintf('\n')
                              
            end
            %Change phase to new optimized value
            phase=phaseOptimizer(indexPhase)*phase;
        end
        
        
        
        % Optimalize amplitude
        if(amplitudeOptimizerOn)
            % create empty array
            evaluationOfAmplitudeOptimizer=zeros(1,length(amplitudeOptimizer));
            
            for(k=1:length(amplitudeOptimizer))
                % generate signal with modified damping constant
                % sig=A*exp(alpha*(0:n-1)).*cos(w*(0:n-1)+ph);
                
                optimizedSignal=A*amplitudeOptimizer(k)*exp(alpha*(0:n-1)).* cos(w*(0:n-1)+(phase));
                
                % create "evaluation" for each modification of amplitude
                evaluationOfAmplitudeOptimizer(k)=sum(abs(sig-optimizedSignal(:)));
            end
            
            % find the best value of amplitudeOptimizer (min of evaluation function)
            [y, indexAmplitude]=min(evaluationOfAmplitudeOptimizer);
            
            if(displayOn)
                fprintf('\t\tamplitude optimization\n')
                fprintf('\t\t\tamplitude optimizer = %f\n',amplitudeOptimizer(indexAmplitude))
                fprintf('\t\t\tamplitude is changed from %f to %f\n', A, amplitudeOptimizer(indexAmplitude)*A)
                fprintf('\n')
            end
            %Change Amplitude to new optimized value
            A=amplitudeOptimizer(indexAmplitude)*A;
        end
        
        
        
        % check, if the optimized values are already good enough
        % The optimalizer is completed only if index reached maxIndex value
        % or when all the optimalizers are equal to 1 (that means the
        % values of all parameters were not changed so it is completed)
        optimizationCompleted = true;
        
        if(omegaOptimizerOn)
            if(omegaOptimizer(indexOmega) ~= 1)
                % optimizer is still not the best
                optimizationCompleted = false;
            end
        end
        
        if(alphaOptimizerOn)
            if(alphaOptimizer(indexAlpha) ~= 1)
                % optimizer is still not the best
                optimizationCompleted = false;
            end
        end
        
        if(phaseOptimizerOn)
            if(phaseOptimizer(indexPhase) ~= 1)
                % optimizer is still not the best
                optimizationCompleted = false;
            end
        end
        
        if(amplitudeOptimizerOn)
            if(amplitudeOptimizer(indexAmplitude) ~= 1)
                % optimizer is still not the best
                optimizationCompleted = false;
            end
        end
        if(optimizationCompleted)
            
            break;
        end
        
        sigRO=A*exp(alpha*(0:n-1)) .* cos(w*(0:n-1)+phase);

%         %Plot graph in each cycle with information about parameters
%         figure
%         plot(0:n-1,sig,'r.-',0:n-1,sigRO,'b.-');        
%         title(['Original signal vs reconstructed signal in ' num2str(index) '. cycle']);
%         annotation('textbox', [0.65, 0.15, 0.1, 0.1], 'String',...
%             "A          changed from " + parameters(1) + "  to  " + A+ newline +...
%             "alpha    changed from " + parameters(2) + "  to  " + alpha+ newline +...
%             "omega  changed from " + parameters(3) + "  to  " + w+ newline +...
%             "phase   changed from " + parameters(4) + "  to  " + phase)


        % plot graph in each cycle with zoomed parts
        % upper graph
        figure
        subplot(2,1,1)
        plot(0:n-1,sig,'r.-',0:n-1,sigRO,'b.-');
        title(['Original signal vs reconstructed signal in ' num2str(index) '. cycle']);
        
        
        
        zoomLevel = 1000;     % variable to determine length of zoom on the x axis
        
        % Graph bottom left
        subplot(2,4,5)
        plot(0:n-1,sig,'r.-',0:n-1,sigRO,'b.-');
        xlim([0 zoomLevel])
        title('beginning of the signal');
        
        % graph bottom middle-left
        subplot(2,4,6)
        plot(0:n-1,sig,'r.-',0:n-1,sigRO,'b.-');
        xlim([length(sig)/3 length(sig)/3 + zoomLevel])
        title('1/3 of the signal');
        
        % graph bottom middle-right
        subplot(2,4,7)
        plot(0:n-1,sig,'r.-',0:n-1,sigRO,'b.-');
        xlim([length(sig)/3 *2 length(sig)/3 *2 + zoomLevel])
        title('2/3 of the signal');
        
        % Graph bottom right
        subplot(2,4,8)
        plot(0:n-1,sig,'r.-',0:n-1,sigRO,'b.-');
        xlim([length(sig) - zoomLevel  length(sig)])
        title('ending of the signal');

         % check, if the optimized values are already good enough
        % The optimalizer is completed only if index reached maxIndex value
        % or when all the optimalizers are equal to 1 (that means the
        % values of all parameters were not changed so it is completed)
        optimizationCompleted = true;
        
        if(omegaOptimizerOn)
            if(omegaOptimizer(indexOmega) ~= 1)
                % optimizer is still not the best
                optimizationCompleted = false;
            end
        end
        
        if(alphaOptimizerOn)
            if(alphaOptimizer(indexAlpha) ~= 1)
                % optimizer is still not the best
                optimizationCompleted = false;
            end
        end
        
        if(phaseOptimizerOn)
            if(phaseOptimizer(indexPhase) ~= 1)
                % optimizer is still not the best
                optimizationCompleted = false;
            end
        end
        
        if(amplitudeOptimizerOn)
            if(amplitudeOptimizer(indexAmplitude) ~= 1)
                % optimizer is still not the best
                optimizationCompleted = false;
            end
        end
        if(optimizationCompleted)
            
            break;
        end
        
        index=index+1;
        if(index>maxIndex)
            break;
        end
        
        
        
    end
end

% Optimized parameters
parametersO = [A alpha w phase];

% reconstructedOptimized signal
sigRO=A*exp(alpha*(0:n-1)) .* cos(w*(0:n-1)+phase);

if(displayOn)
    fprintf('\tOptimized parameters are\n');
    fprintf('\tA     = %.10f\n',parametersO(1));
    fprintf('\talpha = %.10f\n',parametersO(2));
    fprintf('\tw     = %.10f\n',parametersO(3));
    fprintf('\tphase = %.10f\n',parametersO(4));
    fprintf('\n');
end
% =========================================================================

%% plot signal with zoomed parts
% upper graph
figure
subplot(2,1,1)
plot(0:n-1,sig,'r.-');
title('\fontsize{15}Original signal');



zoomLevel = 1000;     % variable to determine length of zoom on the x axis

% Graph bottom left
subplot(2,4,5)
plot(0:n-1,sig,'r.-');
xlim([0 zoomLevel])
title('\fontsize{15}beginning of the signal');

% graph bottom middle-left
subplot(2,4,6)
plot(0:n-1,sig,'r.-');
xlim([length(sig)/3 length(sig)/3 + zoomLevel])
title('\fontsize{15}1/3 of the signal');

% graph bottom middle-right
subplot(2,4,7)
plot(0:n-1,sig,'r.-');
xlim([length(sig)/3 *2 length(sig)/3 *2 + zoomLevel])
title('\fontsize{15}2/3 of the signal');

% Graph bottom right
subplot(2,4,8)
plot(0:n-1,sig,'r.-'    );
xlim([length(sig) - zoomLevel  length(sig)])
title('\fontsize{15}ending of the signal');

% %% plot evaluation graphs
% 
% %omega
% figure
% subplot(2,1,1)
% plot(omegaOptimizer);
% xlim([0  length(evaluationOfOmegaOptimizer)])
% title('Omega Optimizer');
% subplot(2,1,2)
% plot(evaluationOfOmegaOptimizer);
% xlim([0  length(evaluationOfOmegaOptimizer)])
% title('Evaluation of Omega optimizer');
% 
% %alpha
% figure
% subplot(2,1,1)
% plot(alphaOptimizer);
% xlim([0  length(evaluationOfOmegaOptimizer)])
% title('Alpha Optimizer');
% subplot(2,1,2)
% plot(evaluationOfAlphaOptimizer);
% xlim([0  length(evaluationOfOmegaOptimizer)])
% title('Evaluation of Alpha optimizer');
% 
% 
% %phase
% figure
% subplot(2,1,1)
% plot(phaseOptimizer);
% xlim([0  length(evaluationOfPhaseOptimizer)])
% title('Phase Optimizer');
% subplot(2,1,2)
% plot(evaluationOfPhaseOptimizer);
% xlim([0  length(evaluationOfPhaseOptimizer)])
% title('Evaluation of Phase optimizer');
% 
% 
% %amplitude
% figure
% subplot(2,1,1)
% plot(amplitudeOptimizer);
% xlim([0  length(evaluationOfAmplitudeOptimizer)])
% title('Amplitude Optimizer');
% subplot(2,1,2)
% plot(evaluationOfAmplitudeOptimizer);
% xlim([0  length(evaluationOfAmplitudeOptimizer)])
% title('Evaluation of Amplitude optimizer');
%end