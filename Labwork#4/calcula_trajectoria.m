
%% DESCRIÇÃO: Implementa a equação polinomial  de 3ª ordem com componentes de posição e velocidade;
% garante que a trajectória satisfaz uma posição e velocidade final
% desejada.
% - Continuidade e suavidade nas velocidades e acelerações das juntas;
% - Evitar solicitações desmesuradas e irregulares nos actuadores.
%###################################################################################################
% ARGUMENTOS: 
%       - <t> : variável simbólica para o tempo;
%
%       - <t0> : instante inicial (t = t0);
%
%       - <theta0> : valor das juntas na posição inicial (t = t0);
%
%       - <thetaf> : valor das juntas na posição final;
%
%       - <delta_t> : diferença entre tempo no instante final e tempo no
%       instante inicial;
%
%       - <v_juntas0> : velocidade das juntas na posição inicial (com t = t0)
%
%       - <v_juntasf> : velocidade dsa juntas na posição final
%
%###################################################################################################

function [posicao] = calcula_trajectoria(t, t0, theta0, thetaf, delta_t, v_juntas0, v_juntasf)

posicao =                                                                              ...
                                                                                       ...
theta0 +                                                                               ... % a0
v_juntas0*(t - t0) +                                                                   ... % a1*t
((3/delta_t^2)*(thetaf-theta0)-(2/delta_t)*v_juntas0-(1/delta_t)*v_juntasf)*(t-t0)^2 - ... % a2*t^2
((2/delta_t^3)*(thetaf-theta0)-(1/delta^2)*(v_juntasf-v_juntas0))*(t-t0)^3;                % a3*t^3

end