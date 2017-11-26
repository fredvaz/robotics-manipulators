clear all
close all
clc

disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
disp('%%    [Robotica - 10/10/2017 ~ 05/11/2017] LABWORK#2 - PROBLEMA 5   %%')
disp('%%                                                                  %%')
disp('%%                   Frederico Vaz, n. 2011283029                   %%')
disp('%%                   Paulo Almeida, n. 2010128473                   %%')
disp('%%                                                                  %%')
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')

%% Problema 5 - manipulador RRP equipado com uma garra esférica


syms theta1 theta2 d3 theta4 theta5 theta6
L1 = 0.25;
%joint type
R = 1; %junta de rotação
P = 0; %junta prismática

disp('############################################################################################')
% Manipulador [RRP + RRR] - Matriz dos parametros de Denavith-Hartenberg: PJ_DH
disp(' ')
disp('Matriz dos parametros de Denavith-Hartenberg:')
disp('____________________________________________________________________________________')

%           thetai  di     ai  alphai  offseti  jointtypei
PJ_DH = [        0  d1      0   -pi/2       0           P;    % Junta prismática
              pi/2  d2      0       0       0           P;    % Junta prismática
            theta3   0      0   -pi/2       0           R;    % Punho esferico
            theta4   0      0    pi/2       0           R;    % Punho esferico
            theta5  L1      0       0   -pi/2           R];   % Punho esferico
disp('____________________________________________________________________________________')
%parametros de Denavit-Hartenberg
disp(PJ_DH)

disp('############################################################################################')

%%
% Valores de Juntas - "Home"

        q = [1 1 0 0 0];

%%
        