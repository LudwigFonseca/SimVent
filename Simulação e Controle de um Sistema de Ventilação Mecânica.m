%% Variáveis
clc
clear
close all

s   = tf('s');
CL  = 0.05;   % l/cmH2O
RL  = 5 ;     % cmH2O/l/s
CT  = 0.0018; % l/cmH2O
tau = 20e-3;  % s

% Função de transferência que descreve a dinâmica da válvula
VL       = 1 / (tau*s + 1);
VL_roots = roots([tau 1]);

% Função de transferência que descreve o fluxo pulmonar
QL          = ( 1/(RL*CT) ) / ( s + ( 1/CL + 1/CT)*( 1/RL ) );
MA_Inlet    = VL * QL;
MWMA_Inlet  = 5555 / ( (s + 50) * (s + 115.1) );
Z_Inlet     = zero(MWMA_Inlet);
P_Inlet     = pole(MWMA_Inlet);

% Função de transferência que descreve a pressão pulmonar
PP          = ( 1/CT * ( s + ( 1/(CL * RL) ) ) ) / (s^2 + s * ( 1/CL + 1/CT )*( 1/RL ));
MA_Outlet   = VL * PP;
MWMA_Outlet = 2780*(s + 3.9993) / ( s * (s + 50) * (s + 115.1) );
Z_Outlet    = zero(MWMA_Outlet);
P_Outlet    = pole(MWMA_Outlet);

%% Dimensionamento do Controlador da válvula inspiratória
% set(gcf,'renderer','opengl','PaperOrientation','landscape','PaperType','a4','PaperPosition',[-1.25,-0.16,14.00,8.58]); print('-opengl','-cmyk','-r600','-dpdf','F:\Fotos\Inspiratoria\Polo_Zero_Inspiracao2[MATLAB]')
% set(gcf,'renderer','opengl','PaperOrientation','landscape','PaperType','a4','PaperPosition',[-1.25,-0.16,14.00,8.58]); print('-opengl','-cmyk','-r600','-dpdf','F:\Fotos\Inspiratoria\Polo_Zero_Inspiracao[MATLAB]')
% set(gcf,'renderer','opengl','PaperOrientation','landscape','PaperType','a4','PaperPosition',[-1.25,-0.16,14.00,8.58]); print('-opengl','-cmyk','-r600','-dpdf','F:\Fotos\Inspiratoria\Step_Inspiracao2[MATLAB]')
% set(gcf,'renderer','opengl','PaperOrientation','landscape','PaperType','a4','PaperPosition',[-1.25,-0.16,14.00,8.58]); print('-opengl','-cmyk','-r600','-dpdf','F:\Fotos\Inspiratoria\Step_Inspiracao[MATLAB]')
% set(gcf,'renderer','opengl','PaperOrientation','landscape','PaperType','a4','PaperPosition',[-1.25,-0.16,14.00,8.58]); print('-opengl','-cmyk','-r600','-dpdf','F:\Fotos\Inspiratoria\Lugar_Raizes_Inspiracao[MATLAB]')

clc
close all

% A válvula inspiratória controla o fluxo de entrada para os pulmões,
% a modalidade de controle é VCV - Ventilação com Volume Controlado sendo
% assim a variável controlada é o Volume inserido nos pulmões e a variável
% manipulada é a vazão de gás

% Especificações
% TA = 3*(1/50) + 1.5*(1/115.1)
% Erro nulo para referência
Mp = 0.1;   % 10 - Máximo Pico
TF = 0.15;  % 150ms - Tempo de Assentamento de Malha Fechada
Ke = 5555;  % Ganho estático da planta

% Controlador
Csi = sqrt( 1 / ( (pi/log(Mp))^2 + 1 ));
Wn = 3 / (Csi*TF);
pd = -Csi*Wn + Wn*sqrt(1 - Csi^2)*i;

% Fases
O1 = rad2deg(atan2(imag(pd), real(pd)                 ));
O2 = rad2deg(atan2(imag(pd), real(pd) - (P_Inlet(2))  ));
O3 = rad2deg(atan2(imag(pd), real(pd) - (P_Inlet(1))  ));

% Fases à Compensar
FaseC = -180 + O1 + O2 + O3;

% Zero do Controlador
z = abs(real(pd) -  imag(pd) / tand(FaseC) );

% Ganho do Controlador
KLR =  ( ( pd + abs(P_Inlet(2)) ) * ( pd + abs(P_Inlet(1)) ) * pd ) / ( ( pd + z ) );
Kc = abs(real(KLR)/Ke);

% Controlador
C1 = ( Kc * ( s + z ) ) / ( s );

% Malha fechada
MF_Inlet  = feedback(C1 * MA_Inlet, 1);

% Routh Hurwitz Plot
figure()
rlocus(C1 * MA_Inlet);
title('Lugar das Raízes')
ylabel('Imaginário')
xlabel('Real')
xlim([-400 100])
set(findall(gcf,'Type','axes'),'FontSize',12,'FontWeight','bold')
set(findall(gcf,'Type','text'),'FontSize',12,'fontWeight','bold')
set(findall(gcf,'Type','line'),'LineWidth',1.5,'MarkerSize',14)
grid on

% Step Response Plot
figure()
step(MA_Inlet)
title('Resposta ao Degrau - Inspiração')
legend({'Malha Aberta'}, 'location', 'northeast')
ylabel('Amplitude')
xlabel('Tempo')
ylim([0 1.4])
set(findall(gcf,'Type','axes'),'FontSize',12,'FontWeight','bold')
set(findall(gcf,'Type','text'),'FontSize',12,'fontWeight','bold')
set(findall(gcf,'Type','line'),'LineWidth',1.5,'MarkerSize',14)
grid on

figure()
step(MF_Inlet)
title('Resposta ao Degrau - Inspiração')
legend('Malha Fechada', 'location', 'northeast')
ylabel('Amplitude')
xlabel('Tempo')
ylim([0 1.4])
set(findall(gcf,'Type','axes'),'FontSize',12,'FontWeight','bold')
set(findall(gcf,'Type','text'),'FontSize',12,'fontWeight','bold')
set(findall(gcf,'Type','line'),'LineWidth',1.5,'MarkerSize',14)
grid on

% Pole and Zeros Map
figure()
pzmap(MA_Inlet)
title('Mapa de Polos e Zeros - Inspiração')
legend({'Malha Aberta'}, 'location', 'northwest')
ylabel('Imaginário')
xlabel('Real')
set(findall(gcf,'Type','axes'),'FontSize',12,'FontWeight','bold')
set(findall(gcf,'Type','text'),'FontSize',12,'fontWeight','bold')
set(findall(gcf,'Type','line'),'LineWidth',1.5,'MarkerSize',14)
grid on

figure()
pzmap(MF_Inlet);
title('Mapa de Polos e Zeros - Inspiração')
legend({'Malha Fechada'},  'location', 'northwest')
ylabel('Imaginário')
xlabel('Real')
set(findall(gcf,'Type','axes'),'FontSize',12,'FontWeight','bold')
set(findall(gcf,'Type','text'),'FontSize',12,'fontWeight','bold')
set(findall(gcf,'Type','line'),'LineWidth',1.5,'MarkerSize',14)
grid on

% Bode Diagram
figure()
bode(MA_Inlet, MF_Inlet)
title('Diagrama de Bode - Inspiração')
legend('Malha Aberta', 'Malha Fechada')
ylabel('Magnitude')
ylabel('Fase')
xlabel('Frequência')
set(findall(gcf,'Type','axes'),'FontSize',12,'FontWeight','bold')
set(findall(gcf,'Type','text'),'FontSize',12,'fontWeight','bold')
set(findall(gcf,'Type','line'),'LineWidth',1.5,'MarkerSize',14)
grid on

%% Dimensionamento do Controlador da válvula expiratória
% set(gcf,'renderer','opengl','PaperOrientation','landscape','PaperType','a4','PaperPosition',[-1.25,-0.16,14.00,8.58]); print('-opengl','-cmyk','-r600','-dpdf','F:\Fotos\Expiratoria\Polo_Zero_Expiracao3[MATLAB]')
% set(gcf,'renderer','opengl','PaperOrientation','landscape','PaperType','a4','PaperPosition',[-1.25,-0.16,14.00,8.58]); print('-opengl','-cmyk','-r600','-dpdf','F:\Fotos\Expiratoria\Polo_Zero_Expiracao2[MATLAB]')
% set(gcf,'renderer','opengl','PaperOrientation','landscape','PaperType','a4','PaperPosition',[-1.25,-0.16,14.00,8.58]); print('-opengl','-cmyk','-r600','-dpdf','F:\Fotos\Expiratoria\Polo_Zero_Expiracao[MATLAB]')
% set(gcf,'renderer','opengl','PaperOrientation','landscape','PaperType','a4','PaperPosition',[-1.25,-0.16,14.00,8.58]); print('-opengl','-cmyk','-r600','-dpdf','F:\Fotos\Expiratoria\Step_Expiracao2[MATLAB]')
% set(gcf,'renderer','opengl','PaperOrientation','landscape','PaperType','a4','PaperPosition',[-1.25,-0.16,14.00,8.58]); print('-opengl','-cmyk','-r600','-dpdf','F:\Fotos\Expiratoria\Step_Expiracao[MATLAB]')
% set(gcf,'renderer','opengl','PaperOrientation','landscape','PaperType','a4','PaperPosition',[-1.25,-0.16,14.00,8.58]); print('-opengl','-cmyk','-r600','-dpdf','F:\Fotos\Expiratoria\Lugar_Raizes_Expiracao[MATLAB]')

clc
close all

% A válvula expiratória controla a pressão residual nos pulmões do
% paciente, idependente da modalidade ventilatória escolhida. Portanto a
% variável controlada é a pressão ao final da Fase Expiratória (PEEP) e a variável
% manipulada nesse caso é a abertura da válvula

% TA = É uma rampa, não tem tempo de assentamento de MA
% Erro nulo para referência
Mp = 0.1;   % 10 - Máximo Pico
TF = 0.15;  % 150ms - Tempo de Assentamento de Malha Fechada
Ke = 27778;  % Ganho estático da planta

% Controlador
Csi = sqrt( 1 / ( (pi/log(Mp))^2 + 1 ));
Wn = 3 / (Csi*TF);
pd = -Csi*Wn + Wn*sqrt(1 - Csi^2)*i;

% Fases
O1 = rad2deg(atan2(imag(pd), real(pd)                 ));
O2 = rad2deg(atan2(imag(pd), real(pd) - (Z_Outlet)    ));
O3 = rad2deg(atan2(imag(pd), real(pd) - (P_Outlet(3)) ));
O4 = rad2deg(atan2(imag(pd), real(pd) - (P_Outlet(2)) ));

% Fases à Compensar
FaseC = -180 + 2*O1 - O2 + O3 + O4;

% Zero do Controlador
z = abs(real(pd) -  imag(pd) / tand(FaseC) );

% Ganho do Controlador
KLR =  ( ( pd + abs(P_Outlet(3)) ) * ( pd + abs(P_Outlet(2)) ) * pd^2  ) / ( ( pd + z ) * ( pd + abs(Z_Outlet) ) );
Kc = abs(real(KLR)/Ke);

% Controlador
C2 = ( Kc * ( s + z ) ) / s;

% Filtro de Referência
Fr = (4/4.702)*(s + 4.702) / (s + 4);

% Malha fechada
MF_Outlet = feedback(C2 * MA_Outlet, 1);

% Routh Hurwitz Plot
figure()
rlocus(C2*MA_Outlet);
title('Lugar das Raízes')
ylabel('Imaginário')
xlabel('Real')
ylim([-40 40])
xlim([-180 10])
set(findall(gcf,'Type','axes'),'FontSize',12,'FontWeight','bold')
set(findall(gcf,'Type','text'),'FontSize',12,'fontWeight','bold')
set(findall(gcf,'Type','line'),'LineWidth',1.8,'MarkerSize',14)
grid on

% Step Response Plot
figure()
step(MA_Outlet);
title('Resposta ao Degrau - Fase Expiratória')
legend('Malha Aberta')
xlabel('Tempo')
ylabel('Amplitude')
set(findall(gcf,'Type','axes'),'FontSize',12,'FontWeight','bold')
set(findall(gcf,'Type','text'),'FontSize',12,'fontWeight','bold')
set(findall(gcf,'Type','line'),'LineWidth',1.5,'MarkerSize',14)
grid on

figure()
step(MF_Outlet, Fr*MF_Outlet);
title('Resposta ao Degrau - Fase Expiratória')
legend('Malha Fechada s/ Filtro de Ref.', 'Malha Fechada c/ Filtro de Ref.')
xlabel('Tempo')
ylabel('Amplitude')
xlim([0 0.4])
set(findall(gcf,'Type','axes'),'FontSize',12,'FontWeight','bold')
set(findall(gcf,'Type','text'),'FontSize',12,'fontWeight','bold')
set(findall(gcf,'Type','line'),'LineWidth',1.5,'MarkerSize',14)
grid on

% Pole and Zeros Map
figure()
pzmap(MA_Outlet)
title('Mapa de Polos e Zeros - Fase Expiratória')
legend({'Malha Aberta'}, 'location', 'northwest')
ylabel('Imaginário')
xlabel('Real')
set(findall(gcf,'Type','axes'),'FontSize',12,'FontWeight','bold')
set(findall(gcf,'Type','text'),'FontSize',12,'fontWeight','bold')
set(findall(gcf,'Type','line'),'LineWidth',1.5,'MarkerSize',14)
grid on

figure()
pzmap(MF_Outlet)
title('Mapa de Polos e Zeros - Fase Expiratória')
legend({'Malha Fechada s/ Filtro de Ref.'}, 'location', 'northwest')
ylabel('Imaginário')
xlabel('Real')
xlim([-180 0])
set(findall(gcf,'Type','axes'),'FontSize',12,'FontWeight','bold')
set(findall(gcf,'Type','text'),'FontSize',12,'fontWeight','bold')
set(findall(gcf,'Type','line'),'LineWidth',1.5,'MarkerSize',14)
grid on
ax = axes;
set(ax, 'units', 'normalized', 'position',[0.67,0.21,0.2,0.3])
box(ax, 'on')
pzmap(MF_Outlet)
ylabel('Imaginário')
xlabel('Real')
title('')
set(ax, 'xlim',[-6 -2],'ylim',[-1 1])
set(findall(gcf,'Type','line'),'LineWidth',1.5,'MarkerSize',14)
grid on

figure()
pzmap(Fr*MF_Outlet)
title('Mapa de Polos e Zeros - Fase Expiratória')
legend({'Malha Fechada c/ Filtro de Ref.'}, 'location', 'northwest')
ylabel('Imaginário')
xlabel('Real')
xlim([-180 0])
set(findall(gcf,'Type','axes'),'FontSize',12,'FontWeight','bold')
set(findall(gcf,'Type','text'),'FontSize',12,'fontWeight','bold')
set(findall(gcf,'Type','line'),'LineWidth',1.5,'MarkerSize',14)
grid on
ax = axes;
set(ax, 'units', 'normalized', 'position',[0.67,0.21,0.2,0.3])
box(ax, 'on')
pzmap(Fr*MF_Outlet)
ylabel('Imaginário','FontSize',0.1)
xlabel('Real','FontSize',0.1)
title('')
set(ax, 'xlim',[-6 -2],'ylim',[-1 1])
set(findall(gcf,'Type','line'),'LineWidth',1.5,'MarkerSize',14)
grid on

% Bode Diagram
figure()
bode(MA_Outlet, MF_Outlet, Fr*MF_Outlet)
title('Diagrama de Bode - Fase Expiratória')
legend('Malha Aberta', 'Malha Fechada s/ Filtro de Ref.', 'Malha Fechada c/ Filtro de Ref.')
ylabel('Magnitude')
ylabel('Fase')
xlabel('Frequência')
set(findall(gcf,'Type','axes'),'FontSize',12,'FontWeight','bold')
set(findall(gcf,'Type','text'),'FontSize',12,'fontWeight','bold')
set(findall(gcf,'Type','line'),'LineWidth',1.5,'MarkerSize',14)
grid on

%%
% set(gcf,'renderer','opengl','PaperOrientation','landscape','PaperType','a4','PaperPosition',[-1.15,-0.16,14.00,8.58]); print('-opengl','-cmyk','-r600','-dpdf','F:\Fotos\Comparativo Ensaios\Comparativo_Ensaio1[MATLAB]')
% set(gcf,'renderer','opengl','PaperOrientation','landscape','PaperType','a4','PaperPosition',[-1.15,-0.16,14.00,8.58]); print('-opengl','-cmyk','-r600','-dpdf','F:\Fotos\Comparativo Ensaios\Comparativo_Ensaio2[MATLAB]')
% set(gcf,'renderer','opengl','PaperOrientation','landscape','PaperType','a4','PaperPosition',[-1.15,-0.16,14.00,8.58]); print('-opengl','-cmyk','-r600','-dpdf','F:\Fotos\Comparativo Ensaios\Comparativo_Ensaio3[MATLAB]')
% set(gcf,'renderer','opengl','PaperOrientation','landscape','PaperType','a4','PaperPosition',[-1.15,-0.16,14.00,8.58]); print('-opengl','-cmyk','-r600','-dpdf','F:\Fotos\Comparativo Ensaios\Comparativo_Ensaio4[MATLAB]')
% set(gcf,'renderer','opengl','PaperOrientation','landscape','PaperType','a4','PaperPosition',[-1.15,-0.16,14.00,8.58]); print('-opengl','-cmyk','-r600','-dpdf','F:\Fotos\Comparativo Ensaios\Comparativo_Ensaio5[MATLAB]')
% set(gcf,'renderer','opengl','PaperOrientation','landscape','PaperType','a4','PaperPosition',[-1.15,-0.16,14.00,8.58]); print('-opengl','-cmyk','-r600','-dpdf','F:\Fotos\Comparativo Ensaios\Comparativo_Ensaio6[MATLAB]')
% set(gcf,'renderer','opengl','PaperOrientation','landscape','PaperType','a4','PaperPosition',[-1.15,-0.16,14.00,8.58]); print('-opengl','-cmyk','-r600','-dpdf','F:\Fotos\Comparativo Ensaios\Comparativo_Ensaio7[MATLAB]')
% set(gcf,'renderer','opengl','PaperOrientation','landscape','PaperType','a4','PaperPosition',[-1.15,-0.16,14.00,8.58]); print('-opengl','-cmyk','-r600','-dpdf','F:\Fotos\Comparativo Ensaios\Comparativo_Ensaio8[MATLAB]')

% Padrões
clc
clear
close all

% load('Dados/MIT/Ensaio1-MIT.mat')
% load('Dados/TCC/Ensaio1-TCC.mat')
% str = 'Comparativo Entre Simuladores - Ensaio 1';
% yDFMax = 60; yVTMax = 800; yPLMax = 20;
% Step1 = 30; Step2 = 100; Step3 = 5;

% load('Dados/MIT/Ensaio2-MIT.mat')
% load('Dados/TCC/Ensaio2-TCC.mat')
% str = 'Comparativo Entre Simuladores - Ensaio 2';
% yDFMax = 60; yVTMax = 1200; yPLMax = 35;
% Step1 = 30; Step2 = 200; Step3 = 5;

% load('Dados/MIT/Ensaio3-MIT.mat')
% load('Dados/TCC/Ensaio3-TCC.mat')
% str = 'Comparativo Entre Simuladores - Ensaio 3';
% yDFMax = 100; yVTMax = 600; yPLMax = 35;
% Step1 = 50; Step2 = 100; Step3 = 5;

% load('Dados/MIT/Ensaio4-MIT.mat')
% load('Dados/TCC/Ensaio4-TCC.mat')
% str = 'Comparativo Entre Simuladores - Ensaio 4';
% yDFMax = 90; yVTMax = 700; yPLMax = 50;
% Step1 = 30; Step2 = 100; Step3 = 10;

% load('Dados/MIT/Ensaio5-MIT.mat')
% load('Dados/TCC/Ensaio5-TCC.mat')
% str = 'Comparativo Entre Simuladores - Ensaio 5';
% yDFMax = 60; yVTMax = 500; yPLMax = 30;
% Step1 = 30; Step2 = 100; Step3 = 5;

% load('Dados/MIT/Ensaio6-MIT.mat')
% load('Dados/TCC/Ensaio6-TCC.mat')
% str = 'Comparativo Entre Simuladores - Ensaio 6';
% yDFMax = 80; yVTMax = 600; yPLMax = 40;
% Step1 = 40; Step2 = 100; Step3 = 5;

% load('Dados/MIT/Ensaio7-MIT.mat')
% load('Dados/TCC/Ensaio7-TCC.mat')
% str = 'Comparativo Entre Simuladores - Ensaio 7';
% yDFMax = 80; yVTMax = 400; yPLMax = 50;
% Step1 = 40; Step2 = 100; Step3 = 10;

load('Dados/MIT/Ensaio8-MIT.mat')
load('Dados/TCC/Ensaio8-TCC.mat')
str = 'Comparativo Entre Simuladores - Ensaio 8';
yDFMax = 60; yVTMax = 250; yPLMax = 30;
Step1 = 30; Step2 = 50; Step3 = 5;

Vtarget = 500; PTarget = 5;

tMax            = 12001;
DelivredFlow    = DelivredFlow(1:tMax, 1);
DelivredFlowMIT = DelivredFlowMIT(1:tMax, 1);
VolumeTidal     = VolumeTidal(1:tMax, 1);
VolumeTidalMIT  = VolumeTidalMIT(1:tMax, 1);
LungPressure    = LungPressure(1:tMax, 1);
LungPressureMIT = LungPressureMIT(1:tMax, 1);

minDelivredFlow    = min(DelivredFlow(3001:6001, 1));
maxDelivredFlow    = max(DelivredFlow(3001:6001, 1));
minDelivredFlowMIT = min(DelivredFlowMIT(3001:6001, 1));
maxDelivredFlowMIT = max(DelivredFlowMIT(3001:6001, 1));

minVolumeTidal    = min(VolumeTidal(3001:6001, 1));
maxVolumeTidal    = max(VolumeTidal(3001:6001, 1));
minVolumeTidalMIT = min(VolumeTidalMIT(3001:6001, 1));
maxVolumeTidalMIT = max(VolumeTidalMIT(3001:6001, 1));

minLungPressure = min(LungPressure(3001:6001, 1));
maxLungPressure = max(LungPressure(3001:6001, 1));
minLungPressureMIT = min(LungPressureMIT(3001:6001, 1));
maxLungPressureMIT = max(LungPressureMIT(3001:6001, 1));

VolumeEntregue = maxVolumeTidal - minVolumeTidal;
VolumeEntregueMIT = maxVolumeTidalMIT - minVolumeTidalMIT;

errV = abs(VolumeEntregue - Vtarget) * 100 / Vtarget;
errVMIT = abs(VolumeEntregueMIT - Vtarget) * 100 / Vtarget;

PEEP = minLungPressure;
PEEPMIT = minLungPressureMIT;

errP = abs(PEEP - PTarget) * 100 / PTarget;
errPMIT = abs(PEEPMIT - PTarget) * 100 / PTarget;

DEV = ([VolumeEntregue, errV, PEEP, errP]);
MIT = ([VolumeEntregueMIT, errVMIT, PEEPMIT, errPMIT]);

Resistance              = 5;                                        % hPa/L/s
Compliance              = 50;                                       % mL/hPA
tidal_volume            = 500;                                      % mL
TI                      = 1;                                        % s
IE                      = 2;                                        % s
frequencia_respiratoria = 20;                                       % rpm
T_periodo               = 60 / frequencia_respiratoria;             % s
T_inspiracao            = 1 / (IE+1) * T_periodo;                   % s
T_pausa_inspiratoria    = 0.1;                                      % s
T_expiracao             = T_periodo - T_inspiracao;                 % s
t                       = linspace(0, 12, tMax);                    % s
duration                = (T_inspiracao - T_pausa_inspiratoria);    % s
VolumetricFlowRate      = tidal_volume / 1000 / duration;           % mL/s

cycle_timer  = rem(t, T_periodo);

Tins = cycle_timer < T_inspiracao;
Temp = zeros(1, size(Tins, 2));
for i = 1:size(Tins, 2)-2
    if Tins(1, i) == 0 && Tins(1, i+1) == 1
        Temp(i+1) = 1;
    end
end
X_Ins = [0 round(t(1, find(t.*Temp ~= 0)))];

Texp = cycle_timer >= T_inspiracao & cycle_timer <= T_expiracao + T_inspiracao;
Temp = zeros(1, size(Texp, 2));
for i = 1:size(Texp, 2)-2
    if Texp(1, i) == 0 && Texp(1, i+1) == 1
        Temp(i+1) = 1;
    end
end
X_Exp = round(t(1, find(t.*Temp ~= 0)));

% % % % % % % Gráficos % % % % % % %
figure()
subplot(3,1,1)
title(str)
PLT(-yDFMax, 2*yDFMax, X_Exp, X_Ins, TI, IE)
hold on
plot(t, DelivredFlow, 'LineWidth', 2)
plot(t, DelivredFlowMIT, '--', 'LineWidth', 2)
legend({'Fase Inspiratória', 'Fase Expiratória', 'Vazão Entregue', 'Vazão Entregue - MIT'}, 'Location', 'northeastoutside')
ylabel('Fluxo (L/min)', 'FontSize', 12)
ylim([-yDFMax yDFMax])
yticks([-yDFMax:Step1:yDFMax])
grid on
grid minor

subplot(3,1,2)
PLT(0, yVTMax, X_Exp, X_Ins, TI, IE)
hold on
plot(t, VolumeTidal, 'LineWidth', 2)
plot(t, VolumeTidalMIT, '--', 'LineWidth', 2)
legend({'Fase Inspiratória', 'Fase Expiratória', 'Volume Corrente', 'Volume Corrente - MIT'}, 'Location', 'northeastoutside')
ylabel('Volume (mL)')
ylim([0 yVTMax])
yticks([0:Step2:yVTMax])
grid on
grid minor

subplot(3,1,3)
PLT(0, yPLMax, X_Exp, X_Ins, TI, IE)
hold on
plot(t, LungPressure, 'LineWidth', 2)
plot(t, LungPressureMIT, '--', 'LineWidth', 2)
legend({'Fase Inspiratória', 'Fase Expiratória', 'Pressão Pulmonar', 'Pressão Pulmonar - MIT'}, 'Location', 'northeastoutside')
ylabel('Pressão (cmH2O)')
ylim([0 yPLMax])
yticks([0:Step3:yPLMax])
xlabel('Tempo (s)')
grid on
grid minor

% set(findall(gcf,'Type','axes'),'GridAlpha',1,'MinorGridAlpha',1)
set(findall(gcf,'Type','axes'),'FontSize',12,'FontWeight','bold')
set(findall(gcf,'Type','text'),'FontSize',12,'fontWeight','bold')
set(findall(gcf,'Type','line'),'LineWidth',1.5)

function PLT(Ymin, Ymax, X_Exp, X_Ins, TI, IE)
    for i = 1:size(X_Exp, 2)
        rectangle('Position', [X_Ins(i) Ymin TI Ymax], 'FaceColor', [0.85, 0.85, 0.85, 0.5], 'EdgeColor', 'none');
        rectangle('Position', [X_Exp(i) Ymin IE Ymax], 'FaceColor', [0.5020, 0.7020, 1, 0.4], 'EdgeColor', 'none');
    end
    I = line(NaN, NaN, 'LineWidth', 2, 'Color', [0.85, 0.85, 0.85, 0.5]);
    E = line(NaN, NaN, 'LineWidth', 2, 'Color', [0.5020, 0.7020, 1, 0.4]);
    legend(I, 'Inspiração');
    legend(E, 'Fase Expiratória');
end
