
%%
clear all
close all
clc




disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
disp('%%                                                                  %%')
disp('%%          [Robótica - 07/11/2017 ~ 26/11/2017] LABWORK#4          %%')
disp('%%                                                                  %%')
disp('%%                   Frederico Vaz, nº 2011283029                   %%')
disp('%%                   Paulo Almeida, nº 2010128473                   %%')
disp('%%                                                                  %%')
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
disp('')
disp('               INTRODUÇÃO AO PLANEAMENTO DE TRAJECTÓRIAS              ')
disp('')
disp('*************************** Exercício 1 ******************************')

%% Planear	a	trajetória	de	um	manipulador	RPR

syms theta1 d2 theta3

% Tempo:
tf = 8; % tempo final em [segundos], tempo no ponto B

% Comprimentos dos elos:
L2 = 10;

% Junta Rotacional ou Prismatica:
R = 1; P = 0;
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

<<<<<<< Updated upstream
=======

%% POSIÇÃO HOME:



% % O manipulador encontra-se com a configuração "esticado" completamente na
% % vertical e com as juntas prismáticas na configuração mínima;
% 
% bTf = [1  0  0  0  ;
%        0  1  0  0  ;
%        0  0  1  tzh;
%        0  0  0  1  ;]
% 
% Cinemática Inversa:
[ q_home ] = inverse_kinematics_ex3(oTg)

>>>>>>> Stashed changes

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


<<<<<<< Updated upstream
%% VARIÁVEIS GLOBAIS 

% Inicialização do vector de juntas na nossa posição "home" a começar no
% no ponto inicial [ bmin, -30, 35] dado no enuciado 

% POSIÇÃO HOME:
bTf = [  0  -cos(alfa)  sin(alfa)  40;
         0   sin(alfa)  cos(alfa)  20; 
         0   0          0           0;  
         0   0          0           1  ];

[ q ] = inverse_kinematics_ex3(bTf, 0);

q = [ q(1:3) 0 ];

% Juntas em symbolic p/ resolver o Jacobiano
q_aux = [ theta1 d2 theta3 ];

% Construir jacobiana 2 partir dos parâmetros calculados na cinemática inversa
Jac = Jacobian(oTg, Ti, q_aux, PJ_DH(:,6));

% Componentes de velocidade objectivo [ vx vy wz ]
Jac_ = [ Jac(1:2, 1:3); Jac(6,1:3) ];
=======
%% VELOCIDADES

% Inicialização do vector de juntas
 q = [ 0 0 0 ];

 
% Juntas em symbolic p/ resolver o Jacobiano Analítico
q_aux = [ theta1 d2 theta3 ];

% Construir Jacobiano Analítico a partir dos parâmetros calculados na cinemática inversa
Jac = Jacobian(oTg, Ti, q_aux, PJ_DH(:,6));

% retirar as componentes de velocidade nula [ vz wx wy ]
Jac_ = [ Jac(1:2,:); Jac(6,:) ];
        
% Restrição na velocidade linear em y

        
>>>>>>> Stashed changes


%% MENU ("main")

% Variaveis MENU
select = 0;
select2 = 0;
STOP = 5;
STOP2 = 3;

while(select ~= STOP)
    
    select = menu('Seleccione a acao a realizar:', 'Cinematica Directa',...
                                                   'Plot do Robo',...
                                                   'alinea 3)',...
                                                   'Mover Robot',...
                                                   'Quit');  
                                                
    % Matriz dos parametros de Denavith-Hartenberg: PJ_DH
    if select == 1  
        disp('______________________________________________________________________')
        disp(' ')
        disp('Modelo Cinemático Directo do manipulador recorrendo aos parâmetros de D-H :')
        disp('______________________________________________________________________')
        disp(' ')
        PJ_DH_ = SerialLink(L, 'name', 'Robot Planar RRPRP')
        disp(' ')
        disp('______________________________________________________________________')
        disp(' ')
<<<<<<< Updated upstream
        disp('a) oTg: Cinematica Directa c/ variaveis simbolicas:')
=======
        disp('oTg: Cinematica Directa c/ variaveis Simbolicas:')
>>>>>>> Stashed changes
        disp('______________________________________________________________________')
        disp(' ')
        disp(oTg)
        disp(' ')
        disp('______________________________________________________________________')
    disp('#######################################################################')   
    end  
    
    %% PLOT DO ROBOT:
    
    if select == 2
        figure('units','normalized','outerposition',[0 0 1 1]);
         
        
        % Side view ------------------------------------
        subplot(1,2,1);
<<<<<<< Updated upstream
        robot.plot(q, 'workspace', [-10 90 -10 90 -10 90], 'reach', ... 
                       1, 'scale', 10, 'zoom', 0.25); % 'view', 'top', 'trail', 'b.');
                   
        % Prespectiva de topo do Robot -------------------------------------
        subplot(1,2,2);
        robot.plot(q, 'workspace', [-10 90 -10 90 -10 90],...
                      'reach', 1,...
                      'scale', 10,...
                      'zoom', 0.25,...
                      'view',...
                      'top'); % 'trail', 'b.');
=======
        robot.teach(q, 'workspace', [-10 90 -10 90 -10 90],...
                       'reach', 1,...
                       'scale', 10,...
                       'zoom', 0.25);
                   
                   
        % Top view -------------------------------------
        
         subplot(1,2,2);
         robot.plot(q, 'workspace', [-10 90 -10 90 -10 90],...
                       'reach', 1,...
                       'scale', 10,...
                       'zoom', 0.25,...
                       'view',...
                       'top'); % 'trail', 'b.');
>>>>>>> Stashed changes
                   
    disp('#######################################################################') 
    end  

    %% 3) Calcule as expressões para a velocidade das juntas
    if select == 3
        disp('______________________________________________________________________')
        disp(' ')
        disp('b) Expressões para a velocidade das juntas c/ Wz = pi rad/s:')
        disp('______________________________________________________________________')
               
        % Restrição na velocidade Wz
        Wz = pi; 
        
        % Inversa da Jacobiana x Velocidades em 
        qVelocidades = inv(Jac_)*[ 0 0 Wz ]';  
        disp(' ')
        disp('Expressões para a velocidade das juntas c/ Vx = 10cm/s:')
        disp(' ')
        disp(qVelocidades)
      
        
    disp('#######################################################################')    
    end % fim da alinea b)
    
    %% d) Movimento do manipulador com malha de controlo
    
    if select == 4
        
       % sub-menu
       while(select2 ~= STOP2)
           
           select2 = menu('Seleccione a abordagem desejada: ', 'Integrador',...
                                                               'Malha-Fechada',...
                                                               'Back');
           %#######################################################################        
           
           % Velocidades impostas:
           Vx = 0;
           Vy = 0; 
           Wz = pi; % Velocidade angular 
                                         
           % Período de Amostragem dos Controladores 
           h = 0.1;
           
           % Inicializa as Juntas segundo a Matriz Home/Posição Inicial
           alfa_ = 0;
           oTg = eval(subs(bTf, alfa, alfa_));
           
           [ q_controlo ] = inverse_kinematics_ex3(oTg, alfa_);
                   
           % 1. Abordagem Integradora
           if select2 == 1
               
               k = 1;
               
               while(k < 51) 
                   
                   % [ Vx Vy Wz ]
                   V(k,:) = [ Vx Vy Wz ];
                   
                   % Inversa do Jacobiano x Velocidades em XYZ
                   qVelocidades_ = inv(Jac_)*V(k,:)';
                   % Calculo da Inversa do Jacobiano
                   qVelocidades(:,k) = eval(subs(qVelocidades_, q_aux,...
                                                 q_controlo(k,:) ));
                   
                   % Proximas Juntas segundo a Lei de Controlo: Abordagem Integradora
                   q_controlo(k+1,:) = q_controlo(k,:) + h*qVelocidades(:,k)';
                   
                   
                   % tx e ty através da Matriz da Cinemática Directa
                   bTf(:,:,k+1) = eval(subs(oTg, q_aux,...
                                                 q_controlo(k,:) ));
                   
                   clc
                   disp(' ')
                   disp(['Loading... ', num2str((k/50)*100), '%'])
                   
                   % Atendendo que a junta correspondente ao gripper é fixo
                   % acrescentamos 0 a ultima junta de forma a trabalharmos na Toolbox 
                   q_out(k,:) = [ q_controlo(k,1:3) 0 ];
                   pos_out(k,:) = bTf(1:3,4,k);
                   
                   k = k + 1;
               end
               
               % PLOT do Robô com velocidades
               plot_robot2(robot, k, V, qVelocidades, q_out, pos_out);
               
           end
           %#######################################################################           
           % 2. Abordagem em malha-fechada
           if select2 == 2
               
               % Controlo Propocional kp > 0.65 Instavél 
               kp = 0.1;
               % Controlo Derivativo
               kd = 0.01;
               
               % Posição Inicial!
               tx_desej = bTf(1,4);
               ty_desej = bTf(2,4);
               
               alfa_ant = alfa_;
               
               % Deslocamento = Velocidade*Período IDEAL!
               dx = Vx*h; 
               dy = Vy*h; 
               
               k = 1;
               
               while(k < 121)
                   
                   % [ Vx vy Vz ]
                   V(k,:) = [ Vx Vy Wz ];
                   
                   % Inversa do Jacobiano x Velocidades em XYZ
                   qVelocidades_ = inv(Jac_)*V(k,:)';
                   % Calculo da Inversa do Jacobiano
                   qVelocidades(:,k) = eval(subs(qVelocidades_, q_aux,...
                                                 q_controlo(k,:) ));
                   
                   % Proximas Juntas: Controlo Malha fechada
                   q_controlo(k+1,:) = q_controlo(k,:) + kp*h*qVelocidades(:,k)';
                   
                   % tx e ty através da Matriz da Cinemática Directa
                   bTf(:,:,k+1) = eval(subs(oTg, q_aux,...
                                                 q_controlo(k,:) ));
                   
                   % ------------------------------------------------------
                   % Guardamos a Posição actual
                   tx_actual = bTf(1,4,k);
                   ty_actual = bTf(2,4,k);
                   alfa_actual = acot(tan(q_controlo(k+1,1) + q_controlo(k+1,3)));
                  
                   % Posição desejada = Posição + Deslocamento IDEAL!
                   tx_desej = tx_desej + dx;
                   ty_desej = ty_desej + dy;
                   
                   % Velocidade Angular Desejada em Z
                   Wz_desej = Wz; % IDEALMENTE é pi rad/s! Queremos manter a vel Wx
                   
                   dx_erro = tx_desej - tx_actual;
                   dy_erro = ty_desej - ty_actual;
                   alfa_erro = Wz*h - (alfa_actual - alfa_ant);
                   
                   alfa_ant = alfa_actual;
                   
                   Vx = dx_erro/h; % kp*dx_erro + kd*dx_erro/h; % dx_erro/h; %
                   Vy = dy_erro/h; % kp*dy_erro + kd*dy_erro/h; % dy_erro/h; %
                   %Wz = alfa_erro/h; % kp*Wz_erro + kd*Wz_erro/h; % Wz_erro/h; %
                   
                   % NOTA: Como estamos a dividir o deslocamento a efectuar
                   % pelo Período temos de imeadiato a velocidade a impor
                   % no caso c/ parte derivativa do controlo, esta
                   % corresponde acaba por corresponder a velocidade a impor
                   %
                   % Controlamos a velocidade Vx e Vy na tentativa de
                   % corrigir o erro em X e Y
                   % ------------------------------------------------------        
                   clc
                   disp(' ')
                   disp(['Loading... ', num2str((k/120)*100), '%'])
                  
                   % Atendendo que a junta correspondente ao gripper é fixo
                   % acrescentamos 0 a ultima junta de forma a trabalharmos na Toolbox 
                   q_out(k,:) = [ q_controlo(k,1:3) 0 ];
                   pos_out(k,:) = bTf(1:3,4,k);
                   
                   k = k + 1;
               end
               
               % PLOT do Robô com velocidades
               plot_robot3(robot, k, V, qVelocidades, q_out, pos_out);
 
           end
           %#######################################################################
           
       end % fim do sub-menu
      
    disp('#######################################################################')   
    end % fim da alinea d)   
end % fim do menu/ fim do exercicio

