function [velocidade_juntas] = calcula_velocidade(theta, T)

% casos particulares:
    % velocidade das juntas no instante inicial e final:
    
velocidade_juntas(1,:) = 0;
velocidade_juntas(size(T),:) = 0;

for i = 2 : 1 : (size(T)-1) % percorre os troços da trajectória
    for k = 1 : 1 : size(theta) % percorre as juntas do manipulador
        
        if sign(q_juntas(i,k) - q_juntas(i,k-1)) ~= sign(q_juntas(i,k+1) - q_juntas(i,k)) % se houver inversão do sinal
            
            velocidade_juntas = 0;
        
        else
            velocidade_juntas = (1/2)*(  ((theta(i,k+1)-theta(i,k))  /  (T(i+1)-T(i)))  +  ...
                                         ((theta(i,k)-theta(i,k-1))  /  (T(i)-T(i-1)))  );
        end
        
        
    end
end
end
