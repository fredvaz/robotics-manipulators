clear all
close all
clc

disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
disp('%%                                                                  %%')
disp('%%          [Robótica - 07/11/2017 ~ 26/11/2017] LABWORK#3          %%')
disp('%%                                                                  %%')
disp('%%                   Frederico Vaz, nº 2011283029                   %%')
disp('%%                   Paulo Almeida, nº 2010128473                   %%')
disp('%%                                                                  %%')
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
disp(' ')


disp('*************************** Exercício 2 ******************************')
%% Robot 5-DOF (3 de rotação e 2 prismáticos): RRPRP



syms theta1 theta2 d3 theta4 d5

% Comprimentos dos elos:
a    = 50;
bmin = 40 ;
c    = 30;
tzh = a + bmin + c;


% Junta Rotacional ou Prismatica:
R = 1; P = 0;
%_________________________________________________________________________________
%          thetai  |  di  |  ai |  alfai | offseti | jointtypei
%_________________________________________________________________________________
PJ_DH = [  theta1      a      0    -pi/2        0           R;   % Junta Rotacional
%_________________________________________________________________________________
           theta2      0      0     pi/2        0           R;   % Junta Rotacional
%_________________________________________________________________________________
                0     d3      0    -pi/2       25           P;   % Junta Prismática
%_________________________________________________________________________________
           theta4      0      0     pi/2        0           R;   % Junta Rotacional
%_________________________________________________________________________________
                0     d5      0        0       10           P ]; % Junta Prismática
%_________________________________________________________________________________

% A cinematica directa da base   até ao Gripper: 
[ oTg, Ti ] = direct_kinematics(PJ_DH);       

oTg = simplify(oTg);
Ti  = simplify(Ti) ;


%% POSIÇÃO HOME:

% % O manipulador encontra-se com a configuração "esticado" completamente na
% % vertical e com as juntas prismáticas na configuração mínima;
% 
% bTf = [1  0  0  0  ;
%        0  1  0  0  ;
%        0  0  1  tzh;
%        0  0  0  1  ;]
% 
% % Cinemática Inversa:
% [ q_home ] = inverse_kinematics_ex2(bTf);   


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


%% VELOCIDADES

% Inicialização do vector de juntas
q = [ 0 0 0 0 0 ];


% Juntas em symbolic p/ resolver o Jacobiano
q_aux = [ theta1 theta2 d3 theta4 d5 ];
       
% Construir jacobiana a partir dos parâmetros calculados na cinemática inversa
Jac = Jacobian(oTg, Ti, q_aux, PJ_DH(:,6));

% retirar as componentes de velocidade nula [ vz wx wy ]
Jac_ = [ Jac(1:2,:); Jac(6,:) ];
        
% Restrição na velocidade linear em y
Vy = 20; %[cm/seg]
        


%% MENU ("main")

% Variaveis MENU
select = 0;
select2 = 0;
STOP = 7;
STOP2 = 3;

while(select ~= STOP)
    
    select = menu('Seleccione a acção a realizar:', 'Cinemática Directa',...
                                                    'Plot do Robo',...
                                                    'alinea a)',...
                                                    'alinea b)',...
                                                    'alinea c)',...
                                                    'Quit');  
                                                
    % Matriz dos parametros de Denavith-Hartenberg: PJ_DH
    if select == 1  
        disp('______________________________________________________________________')
        disp(' ')
        disp('PJ_DH: Matriz dos parametros de Denavith-Hartenberg:')
        disp('______________________________________________________________________')
        disp(' ')
        PJ_DH_ = SerialLink(L, 'name', 'Robot Planar RRPRP')
        disp(' ')
        disp('______________________________________________________________________')
        disp(' ')
        disp('oTg: Cinematica Directa c/ variaveis simbolicas:')
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
         % Prespectiva de lado do Robot  
        subplot(1,2,1);
        robot.teach(q, 'workspace', [-10 90 -10 90 -10 90], 'reach', ... 
                       1, 'scale', 10, 'zoom', 0.25); % 'view', 'top', 'trail', 'b.');
                   
                   
        % Prespectiva de topo do Robot -------------------------------------
        
         subplot(1,2,2);
         robot.plot(q, 'workspace', [-10 90 -10 90 -10 90],...
                       'reach', 1,...
                       'scale', 10,...
                       'zoom', 0.25,...
                       'view',...
                       'top'); % 'trail', 'b.');
                   
    disp('#######################################################################') 
    end   

    %% a) Calcule a matriz Jacobiana do manipulador ºJ
    if select == 3
        
       


        
        % Inversa da Jacobiana x Velocidades em XYZ 
        qVelocidades = Jac_ \ [ 0 Vy 0 ]';  
        
        
        disp(' ')
        disp('Expresoes para a velocidade das juntas p/ Vx = 10cm/s:')
        disp(' ')
        disp('______________________________________________________________________')
        disp(' ')
        disp(qVelocidades)
        disp(' ')
        disp('______________________________________________________________________')
        
        
        disp(' ')
        disp('#######################################################################')
    end % fim da alínea a)
    
    %% b) Admitindo que se controla unicamente as juntas de rotação theta1,theta2 e a junta prismática b, 
    %     calcule as velocidades de	 funcionamento que permitem seguir a peçaa
    
    if select == 4
        
        %
        
        
        
        % INSERT CODE HERE
        
        
        
        %        
        
    disp('#######################################################################')    
    end % fim da alinea b)
    
    %% c) Calcule as expressões	analíticas das variáveis das juntas	?1,?2 e	b em função	dos	
    %     parâmetros da trajetória pretendida para a garra (x=bmin, y variável).
    
    if select == 5
                
        %
        
        
        
        % INSERT CODE HERE
        
        


        
        %        
        
    disp('#######################################################################')   
    end % fim da alinea c)
    

    %% d) Movimento do manipulador com malha de controlo      [FALTA ADAPTAR]
    
    if select == 6
        
       % sub-menu
       while(select2 ~= STOP2)
           

           % Posição home/inicial do Gripper do Robot
           tx = 0; 
           ty = 0;
           tz = a + b + c;

           
           % Perído de Amostragem
           h = 0.1;
           
           select2 = menu('Seleccione a abordagem desejada: ', 'Integrador',...
                                                               'Malha-Fechada',...
                                                               'Back');
           %#######################################################################                                                    'Back'); 
           % 1. Abordagem Integradora
           if select2 == 1
               
               for i=1:1:10
                   % Vector de Translação
                   P = [ tx ty 10 ];
                   
                   % Manter orientação do Gripper na posição home/inicial
                   % Significa manter a Matriz de Rotação da posição home/inicial
                   oTg_actual = [ bTf(1:3,1:3); P';

                                  0    0   0    1  ];

                   
                   % Obter os angulos para segund a posição
                   [ q ] = inverse_kinematics_ex1(oTg_actual);
                   
                   % Jacobiando das velocidades nas Juntas
                   qVelocidades = eval(subs(qVelocidades, q_aux, q));
                   
                   % Lei de Controlo: Abordagem Integradora
                   q(i+1) = q(i) + h*qVelocidades;
                   
                   
               end
               
               % Teste do Plot
               figure('units','normalized','outerposition',[0 0 1 1]);
               % Prespectiva de lado do Robot
               subplot(1,2,1);
               robot.plot(q, 'workspace', [-10 90 -10 90 -10 90], 'reach', ...
                   1, 'scale', 10, 'zoom', 0.25); % 'view', 'top', 'trail', 'b.');
               % Prespectiva de topo do Robot
               subplot(1,2,2);
               robot.plot(q, 'workspace', [-10 90 -10 90 -10 90], 'reach', ...
                   1, 'scale', 10, 'zoom', 0.25, 'view', 'top'); % 'trail', 'b.');
               %
           end
           %#######################################################################           
           % 2. Abordagem em malha-fechada
           if select2 == 2
               
               %
               
               
               
               % INSERT CODE HERE
               
               
               
               %

               
           end
           %#######################################################################
           
       end % fim do sub-menu
      
 
    disp('#######################################################################')   
    end % fim da alinea d)
    

end % fim do menu/ fim do exercicio

