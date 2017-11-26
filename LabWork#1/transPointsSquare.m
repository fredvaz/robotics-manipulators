
%% Obtenção dos pontos dos Quadrados

function [ square ] = transPointsSquare(T, points)

    for i=1:size(points,1)
        % Matriz 4x4 * Vector 1x4
        aux = T*[points(i,:) 1]';
        square(i,:) = aux(1:3)';
    end

end