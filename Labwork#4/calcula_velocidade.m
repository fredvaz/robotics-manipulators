function [velocidade_juntas] = calcula_velocidade()

if sign() ~= sign()
    velocidade_juntas = 0;
end

else
    velocidade_juntas = (1/2)*(  ((theta(i+1)-theta(i))/(t(i+1)-t(i)))  +  ...
                                 ((theta(i)-theta(i-1))/(t(i)-t(i-1)))  );
end

end