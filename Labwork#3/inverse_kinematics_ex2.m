
%% Função da Cinemática Inversa do Robot
%  Consultar Ex2_Inversa.pdf, para melhor compreensão da implementação

function [ q ] = inverse_kinematics_ex2(oTg)

% vector a
ay = oTg(2,3);
az = oTg(3,3);

% Vector t
tx = oTg(1,4);
ty = oTg(2,4);

%-----------------------------------------
% theta1:
theta1 = atan2(ty,tx);

% theta2:
theta2 = atan2(ay, -az*sin(theta1));

% d3:
d3 = sqrt(tx^2 + ty^2);


q = [theta1 theta2 d3];


% t1 = atan2(tx,ty);
% 
% t2 = atan(tx*cos(th1) + ty*sin(th1), tz - 50);
% 
% b = [cos(th1) * sin(th2) * tx] + [sin(th1) * sin(th2) * ty] + [cos(th2) * (tz - 50)];

end