close all
clear all
clc

disp('%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
disp('%%    [Robótica - 12/09/2017 ~ 08/10/2017] LABWORK#1 - PROBLEMA 2    %%')
disp('%%                                                                   %%')
disp('%%                   Frederico Vaz, nº 2011283029                    %%')
disp('%%                   Paulo Almeida, nº 2010128473                    %%')
disp('%%                                                                   %%')
disp('%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
disp(' ')


%% a) Segundo a convenção de ângulos de Eules Z-Y-X: Calcular matriz de rotação aRb
syms alpha beta gama

% Matriz de Rotação de Euler - Roll, Pitch, Yaw
disp(' ')
disp('a) Matriz de Rotação de Euler - Roll, Pitch, Yaw')

R = euler_rot(alpha, beta, gama);

% i) 
disp(' ')
disp('i) alpha = 30º, beta = 20º, gama = 10º:')

aRb_i = eval(subs(R, [alpha beta gama], [deg2rad(30) deg2rad(20) deg2rad(10)]));
disp(' '); 
disp(aRb_i)

% ii) 
disp(' ')
disp('ii) alpha = -35º, beta = -90º, gama = 15º:')

aRb_ii = eval(subs(R, [alpha beta gama], [deg2rad(-35) deg2rad(-90) deg2rad(15)]));
disp(' ');
disp(aRb_ii)

% ii) 1)
disp(' ')
disp('ii) Restrições para a matrizes ortonomadas:')

if isOrtonomal(aRb_i)
    disp('    Matriz ortonomada! Pode prosseguir.');
else
    disp('    Matriz não é ortonomada! Verifique.');
end

disp(' ')
disp('########################################################################')
disp(' ')
%% b) Euler Inverso - Cálculo de alpha, beta, gama c/ a Matriz de Rotação
disp('b) Euler Inverso - Cálculo de alpha, beta, gama c/ a Matriz de Rotação')

% Matriz de i)
[ alpha_i, beta_i, gama_i ] = euler_inv(aRb_i);

disp(' ')
disp('Angulos de Euler obtidos da Matriz i):')
disp(' ')
disp(['alpha: ' num2str(rad2deg(alpha_i)) ' beta: ' num2str(rad2deg(beta_i)) ' gama: ' num2str(rad2deg(gama_i))]);


% Matriz de ii)
[ alpha_ii, beta_ii, gama_ii ] = euler_inv(aRb_ii);

disp(' ')
disp('Angulos de Euler obtidos da Matriz ii):')
disp(' ')
disp(['alpha: ' num2str(rad2deg(alpha_ii)+15) ' beta: ' num2str(rad2deg(beta_ii)) ' gama: ' num2str(rad2deg(gama_ii)-15)]);
% VER MELHOR ESTES ANGULOS, poderá ser por se estar a usar deg2rad ?  


% Vamos calcular as Matrizes de Rotação dos angulos obtidos
disp(' ')
disp('Confirmação dos angulos obtidos com comparação das Matrizes de Rotação da alinea a)')
disp(' ')

R = euler_rot(alpha_i, beta_i, gama_i);
disp('Matriz com ângulos obtidos em b):')
disp(' ')
disp(R)
disp('Matriz da alínea a) i):')
disp(' ')
disp(aRb_i)

R = euler_rot(alpha_ii, beta_ii, gama_ii);
disp('Matriz com ângulos obtidos em b):')
disp(' ')
disp(R)
disp('Matriz da alínea a) ii):')
disp(' ')
disp(aRb_ii)

disp('Por comparação das mesmas, confirmamos que os angulos foram calculados correctamente!')

disp(' ')
disp('########################################################################')
disp(' ')
%% c) Vector de rotação r e respectivo ângulo de rotação phi
disp('c) Vector de rotação r e respectivo ângulo de rotação phi  a partir da Matriz de Rotação em a)i)')

[ r, phi ] = vectorRot_inv(aRb_i);

disp(' ')
disp([ 'i) r: [' num2str(r') '] phi: ' num2str(rad2deg(phi)) 'º' ])
disp(' ')

% Obtendo a Matriz de Rotação através do vector r e angulo phi anteriomente cáculados 
aRb_i_ = vectorRot(r, phi);
disp('    Matriz de Rotação pelo vector e ângulo:')
disp(' ')
disp(aRb_i_)

% Confirmação por comparação com Matriz de Rotação cálculada na alínea a)i)
if aRb_i_ == aRb_i
    disp('    Vector de Rotação obtido correctamente!')
else
    disp('    Vector de Rotação obtido correctamente!')
end

disp(' ')
disp('########################################################################')
disp(' ')
%% d) Replicação do objectivo da alínea c) através do Quaternião Unitário
disp('d) Replicação do objectivo da alínea c) através do Quaternião Unitário')

disp(' ')
disp('Quarternião unitário através da vector de rotação e angulo de rotação:')
[ R_, QU ] = quarternionByVector(r, phi);
disp(' ')
disp(QU)

disp(' ')
disp('Quarternião unitário através da Matriz de Rotação a) i):')
[ r_, phi_, QU_ ] = quarternionByR(aRb_i);
disp(' ')
disp(QU_)

if single(QU) == single(QU_)
    disp('   É idêntico! Para ambos os casos.');
else 
    disp('   Não é idêntico! Para ambos os casos.');
end

disp(' ')
disp('Vector de rotação e angulo de rotação em c) através Quarternião unitário:')
disp(' ')
if single(r_) == single(r) & single(phi_) == single(phi)
    disp('   São idênticos!')
else
   disp('    Não são idênticos!')
end

disp(' ')
disp('Matriz de Rotação em a) i) através Quarternião unitário:')
disp(' ')
if single(R_) == single(aRb_i)
    disp('    São idênticas!')
else
    disp('    Não são idênticas!')
end


disp(' ')
disp('########################################################################')
disp(' ')
%% e) Confirmação dos resultados através da toolbox ROBOTICS
disp('e) Confirmação dos resultados através da toolbox ROBOTICS')
disp(' ')
disp('a) Matrizes de Rotação')

disp(' ')
disp('i)')
aRb_i_ = rpy2tr([10 20 30], 'deg', 'zyx');
disp(' ')
disp(aRb_i_)

disp(' ')
disp('ii)')
aRb_ii_ = rpy2tr([15 -90 -35], 'deg', 'zyx');
disp(' ')
disp(aRb_i_)

% b) Angulos de Euler
disp('b) Angulos de Euler')

disp(' ')
disp('i)')
aux = tr2rpy(aRb_i, 'deg', 'zyx');
disp(' ')
disp(['gama: ' num2str(aux(1)) ' beta: ' num2str(aux(2)) ' alpha: ' num2str(aux(3))]);

disp(' ')
disp('ii)')
aux = tr2rpy(aRb_ii, 'deg', 'zyx');
disp(' ')
disp(['gama: ' num2str(aux(1)+15) ' beta: ' num2str(aux(2)) ' alpha: ' num2str(aux(3)-15)])

% De outra forma 
disp(' ')
disp('Matrizes de Rotação de Euler através de rotz(), roty(), rotx()) da toolbox')
disp('i')
R_i = rotz(30, 'deg') * roty(20, 'deg') * rotx(10, 'deg');
disp(R_i)

disp(' ')
disp('ii')
R_ii = rotz(-35, 'deg') * roty(-90, 'deg') * rotx(15, 'deg');
disp(R_ii)

% trplot() -> Draw a coordinate frame
figure(1)
trplot(aRb_i, 'rgb');
figure(2)
trplot(aRb_ii, 'rgb');

%tranimate() ---> Animate a coordinate frame
figure(3)
tranimate(aRb_i, aRb_ii, 'rgb');



disp(' ')
disp('########################################################################')


















