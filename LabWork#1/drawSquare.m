
%% Cria um quadrado
% Inputs: points - [ x y x ]4x3, color

function drawSquare(points, color)


fill3(points(:,1), points(:,2), points(:,3), color)
hold on

% Configuração do gráfico
grid on 
xlabel('Eixo dos xx')
ylabel('Eixo dos yy')
zlabel('Eixo dos zz')
axis([-25 35 -10 30 0 15])

view(140, 30);
set(gca, 'DataAspectRatio', [1 1 1])

end