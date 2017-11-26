
%% Função da Cinemática Inversa do Robô
%  Consultar Ex1_Inversa.pdf, para melhor compreensão da implementação 

function [ q ] = inverse_kinematics_ex1(oTg)

    % Cumprimento dos elos
    a2 = 40;
    
    % Vector t
    tx = oTg(1,4);
    ty = oTg(2,4);
    
    % Caso em que theta2 é negativo
    theta2 = -acos((tx^2 + ty^2 - a2^2 - a2^2)/(2*a2*a2));
    theta1 = atan2(ty,tx) - atan2(a2*sin(theta2), a2+a2*cos(theta2));
    theta3 = (pi/2 - theta1 - theta2) - pi/2; % subtraiu-se -pi/2, só para o gripper ficar "alinhado" com os eixos XY
    
    q = [theta1 theta2 theta3];
    
end