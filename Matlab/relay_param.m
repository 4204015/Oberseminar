a=1;    % Amplitude des Relayblocks
h=0;    % Schaltsch
h_offset=0; % gegebenfalls Offset für Hysterese
a_offset=0; % gegebenfalls Offset für Amplitude

%simouts=zeros(1000,3,5);
%touts=zeros(1000,5);

figure(1);
for i=1:1
    h=h+0.4;
    sim('relay_sim.slx',[0.1 100]);
    subplot(5,1,i);
    stairs(tout,simout);
    grid on;
    %ylim([-1.5 1.5]);
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

%% negative inverse Übertragungsfunktion N(a)

N_RE=4*a/pi/amp_dauerschwingung*sqrt(1-(h/amp_dauerschwingung)^2);
N_IM=-4*a/pi/amp_dauerschwingung*h/amp_dauerschwingung;
N=4*a/pi/amp_dauerschwingung*(sqrt(1-(h/amp_dauerschwingung)^2)-h/amp_dauerschwingung*i);
