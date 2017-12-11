clear all
close all
clc

disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
disp('%%                                                                  %%')
disp('%%                   PLANEAMENTO DE TRAJECTÓRIAS                    %%')
disp('%%                                                                  %%')
disp('%%          [Robótica - 28/11/2017 ~ 17/12/2017] LABWORK#4          %%')
disp('%%                                                                  %%')
disp('%%                   Frederico Vaz, nº 2011283029                   %%')
disp('%%                   Paulo Almeida, nº 2010128473                   %%')
disp('%%                                                                  %%')
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
disp(' ')
disp('*************************** Exercício 1 ******************************')

%% VARIÁVEIS GLOBAIS

syms theta1 d2 theta3

% Tempo:
tf = 8; % tempo final em [segundos], tempo no ponto B (fim da trajectória)
ti = 5; % instante de tempo em que o manipulador passa no ponto pi

% Comprimentos dos elos:
L2 = 10;

% Junta Rotacional ou Prismatica:
R = 1; P = 0;


%% MODELO DE CINEMÁTICA DIRECTA DA MANIPULADOR (NOTA: o modelo cinemático directo é semelhante ao robot
% implementado no Labwork#3, ex3)


%_________________________________________________________________________________
%          thetai  |  di  |  ai |  alfai | offseti | jointtypei
%_________________________________________________________________________________
PJ_DH = [  theta1      0      0     pi/2     pi/2           R;   % Junta Rotacional
%_________________________________________________________________________________
                0     d2      0    -pi/2        0           P;   % Junta Prismática
%_________________________________________________________________________________
           theta3      0      0     pi/2        0           R;   % Junta Rotacional
%_________________________________________________________________________________
                0     L2      0        0        0           R; ]; % Indiferente (Não aplicável)
%_________________________________________________________________________________



% A cinematica directa da base   até ao Gripper: 
[ oTg, Ti ] = direct_kinematics(PJ_DH);       

oTg = simplify(oTg);
Ti  = simplify(Ti) ;


%% transformações para os pontos que definem a trajectória do mamnipulador

0_T_A = [ 0   0.9659    0.2588   21.4720 ;
          0   -0.2588   0.9659   22.1791 ;
          1     0         0         0    ;
          0     0         0         1   ];
      
0_T_i = [ 0   0.9659    0.2588   26.2396 ;
          0   -0.2588   0.9659   15.9659 ;
          1     0         0         0    ;
          0     0         0         1   ];
      
0_T_B = [ 0   0.8660     -0.5     12.0   ;
          0    0.5      0.8660   22.5167 ;
          1     0         0         0    ;
          0     0         0         1   ];




%% INICIALIZAÇÃO DO ROBOT: CRIAR LINKS


for i = 1 : size(PJ_DH,1)
    
    if PJ_DH(i,6) == R              % Juntas Rotacionais
        
        L(i) = Link('d',eval(PJ_DH(i,2)),...
                    'a', eval(PJ_DH(i,3)),...
                    'alpha', eval(PJ_DH(i,4)),...
                    'offset', eval(PJ_DH(i,5)));
    end
    
    if PJ_DH(i,6) == P              % Junta Prismática
        
        L(i) = Link('theta',eval(PJ_DH(i,1)),...
                    'a', eval(PJ_DH(i,3)),...
                    'alpha', eval(PJ_DH(i,4)),...
                    'offset', eval(PJ_DH(i,5)),...
                    'qlim', [25 50]);
    end

end

robot = SerialLink(L, 'name', 'Robot Planar RRR');


%% VARIÁVEIS GLOBAIS 


% % POSIÁO HOME:

[ q ] = inverse_kinematics_ex1(oTg)

% 
% % Juntas em symbolic p/ resolver o Jacobiano
% q_aux = [ theta1 d2 theta3 ];
% 
% % Construir jacobiana 2 partir dos parâmetros calculados na cinemática inversa
% Jac = Jacobian(oTg, Ti, q_aux, PJ_DH(:,6));
% 
% % Componentes de velocidade objectivo [ vx vy wz ]
% Jac_ = [ Jac(1:2, 1:3); Jac(6,1:3) ];


%% PLANEAMENTO DO ESPAÇO DAS JUNTAS

% NOTA: o vector <q> representa os valores das juntas 
                   


for t = 0 : 0.01 : 1
    posicao = calcula_trajectoria(t, t0, tf, theta0, thetaf, delta_t, v_juntas0, v_juntasf);
end
                   
                

