clear all;

d=1;    % Amplitude des Relayblocks
e=0;    % Hysterese
e_offset=0; % gegebenfalls Offset für Hysterese
d_offset=0; % gegebenfalls Offset für Amplitude

%simouts=zeros(1000,3,5);
%touts=zeros(1000,5);

figure(1);
n=6;
for i=1:n
    e=e+1;
    sim('relay_sim.slx',[0.1 100]);
    subplot(n,1,i);
    stairs(tout,simout(:,:));
    grid on;
    %ylim([-1.5 1.5]);
    %xlim([0 100]);
    for j=1:size(tout)
        simouts(j,:,i)=simout(j,:);
        touts(j,i)=tout(j);
    end
end
%% Frequenz der Dauerschwingung ermitteln
dif=diff(simouts(:,2,1)); % 
count=0;
xfirst=1;
t_begin=0;
t_end=0;
for i=1:size(tout)-1
    if dif(i)>0 % bei positive Flanke zählen
        count=count+1;
        if(xfirst) % Begin des Zeitraumes
            t_begin=touts(i,1);
            xfirst=0;
        end
        t_end=touts(i,1); % Ende des Zeitraumes
    end
end

T_period=(t_end-t_begin)/count; % Periodendauer
omega_180=2*pi/T_period;           % Freuqenz der Dauerschwingung

%% Amplitude des Eingangssignals des Relay-Glied / Ausgangssignals der Regelstrecke
simout_alt=0;
amp_dauerschwingung=max(simouts(:,3,1));%0;
% for i=1:size(tout)
%     if simouts(i,3,1)>=simout_alt
%         amp_dauerschwingung=simouts(i,3,1);
%     end
% end

%% negative inverse Übertragungsfunktion des Relays N(a), a = amp_dauerschwingung

nr = 4*d/pi/amp_dauerschwingung*sqrt(1-(e/amp_dauerschwingung)^2);
ni = -4*d/pi/amp_dauerschwingung*e/amp_dauerschwingung;
N_RE = real(nr)-image(ni);
N_IM = real(ni)+imag(nr);

%% Parameteridentifikation über G(j*omega_180)=-1/N(a), a = amp_dauerschwingung
% PT1-Strecke
K_pt1 = -1/N_RE;
T_pt1 = N_IM/N_RE/omega_180;

% PT2-Strecke
T1_pt2 = -N_RE/N_IM/omega_180 - sqrt(N_RE/N_IM/omega_180+1/omega_180^2);
T2_pt2 = -N_RE/N_IM/omega_180 + sqrt(N_RE/N_IM/omega_180+1/omega_180^2);
K_pt2 = (T1_pt2*T2_pt2*omega_180^2-1)/N_RE;
