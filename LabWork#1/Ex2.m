%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                      %
%                       LabWork#1 - Exercício 2                        %
%                                                                      %
%                    Frederico Vaz, nº 2011283029                      %
%                    Paulo Almeida, nº 2010128473                      %
%                                                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all
close all
clc

syms alpha beta gamma

%% a) 
disp('a) Matriz de transformação homogénea aTb');
% i
disp('i. ');

P = [1 2 3];
R = rotEuler(alpha, beta, gamma);

aTb = [R P'; 0 0 0 1];

aTb = eval(subs(aTb, [alpha, beta, gamma],[deg2rad(30),deg2rad(20),deg2rad(10)]))

% ii
disp('i. ');

P = [0 2 -1];
R = rotEuler(alpha, beta, gamma);

aTb = [R P'; 0 0 0 1];

aTb = eval(subs(aTb, [alpha, beta, gamma],[deg2rad(20),deg2rad(0),deg2rad(-10)]))


%% b) 
disp('b) ');

% Ponto em B
bP = [1 0 1];
R = rotEuler(alpha, beta, gamma);

aTb = [R bP'; 0 0 0 1];

aTb = eval(subs(aTb, [alpha, beta, gamma],[deg2rad(20),deg2rad(0),deg2rad(0)]))

% Posição de B em relação ao referencial A
aPb = [3 0 1]';

% Ponto P no referencial A
aP = aTb*[bP 1]'





%% c) 





