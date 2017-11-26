clear all
close all
clc

disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
disp('%%    [Robotica - 10/10/2017 ~ 05/11/2017] LABWORK#2 - PROBLEMA 3   %%')
disp('%%                                                                  %%')
disp('%%                   Frederico Vaz, n. 2011283029                   %%')
disp('%%                   Paulo Almeida, n. 2010128473                   %%')
disp('%%                                                                  %%')
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')

%% Problema 3 - Obtenha os parametros de D-H dos 3 manipuladores

disp('************************************** ROBOT1 **********************************************')
syms d1 d2 theta3 theta4 theta5
L1 = 0.25;
%joint type
R = 1; %junta de rotação
P = 0; %junta prismática

%valores do robot (arbitrários)
q = [4 5 0 pi/4 pi/4];

disp('############################################################################################')
% Robot 1 [PP + RRR] - Matriz dos parametros de Denavith-Hartenberg: PJ_DH
disp(' ')
disp('Robot 1 - Matriz dos parametros de Denavith-Hartenberg: PJ_DH')
disp(' ')

%           thetai  di     ai  alphai  offseti  jointtypei
PJ_DH = [        0  d1      0   -pi/2       0           P;    % Junta prismática
              pi/2  d2      0       0       0           P;    % Junta prismática
            theta3   0      0   -pi/2       0           R;    % Punho esferico
            theta4   0      0    pi/2       0           R;    % Punho esferico
            theta5  L1      0       0   -pi/2           R];   % Punho esferico

%parametros de Denavit-Hartenberg
disp(PJ_DH)

disp('############################################################################################')


%%
select = 0;
sair = 4;

while (select ~= sair)
    
    select = menu('Seleccione a acção a realizar:', 'alinea a) & b)',...
                                                    'a) & b) confirmação dos resultados',...
                                                    'alinea c)',...
                                                    'sair do programa');
    
    %% a) & b) Representação grafica dos robots c/ o punho esférico
    if select == 1
        
        
        % Valores Juntas - "home"
        q = [1 1 0 0 0];
        
        %juntas prismáticas ('Link' cria um objecto com os parametros relativos a uma junta e elo do robot)
        for i=1:2
            L(i) = Link('theta', eval(PJ_DH(i,1)), 'a', eval(PJ_DH(i,3)), 'alpha', eval(PJ_DH(i,4)), 'qlim', [0 1]);
        end
        
        %juntas rotacionais
        for i=3:5
            L(i) = Link('d',eval(PJ_DH(i,2)), 'a', eval(PJ_DH(i,3)), 'alpha', eval(PJ_DH(i,4)));
        end
        
        %'SerialLink' cria um robot a partir dos objectos do tipo <Link>
        robot = SerialLink(L, 'name', 'Robot1');
        
        %desenha o robot num ambiente com reguladores para as juntas
        figure('units','normalized','outerposition',[0 0 1 1]);
        robot.teach(q, 'workspace', [-2 2 -2 2 -1 2], 'reach',1); % "drive" a graphical robot using a graphical slider panel
    end
    
    %% a) e b) confirmação dos dados
    if select == 2
        
        %matriz de transformação da "hand"(H) para o "robot"(0), simbólica
        [ T0_H, Ti ] = direct_kinematics(PJ_DH);
        
        % matriz de transformação de T0_H obtida pela função MGD_DH(PJ_DH)(nossa 'direct_kinematics(PJ_DH)')...
        % substituindo as variáveis por valores atribuidos no vector q[];
        T0_H_values = eval(subs(T0_H, [d1 d2 theta3 theta4 theta5], q));
        
        %obter a mesma substituição mas agora usando a função 'fkine' da toolbox em
        %vez da 'direct_kinematics()
        T0_H_Toolbox = robot.fkine(q);
        
    
        disp('Matriz de transformação para cinematica Directa obtida com MGD_DH() c/ variaveis simbolicas:')
        disp('____________________________________________________________________________________')
        T0_H = simplify(T0_H);
        disp(T0_H)
        disp('____________________________________________________________________________________')

        disp(' ')
        disp('Matriz transformação obtida com MGD_DH() e com os valores atribuidos em q[]')
        disp('____________________________________________________________________________________')
        disp(T0_H_values)
        disp('____________________________________________________________________________________')

        disp(' ')
        disp('Matriz de transformação obtida com a toolbox e com os valores artribuidos em q[]')
        disp('____________________________________________________________________________________')
        disp(T0_H_Toolbox)
        disp('____________________________________________________________________________________')

        if(abs((T0_H_values - T0_H_Toolbox)) < 1e-15)
            disp('Os valores obtidos são coincidentes com os obtidos a partir da toolbox');
        else
            disp('As matrizes não coincidem!');
        end
    end
    
    %% c) Modelo inverso dos Robots
    if select == 3
        %obtém Parâmetros da Cinemática Inversa do robot
        PCI = inverse_kinematics_R1(T0_H);
        
        %substitui na matriz simbólica
        T0_H_values_inv = eval(subs(T0_H, [d1 d2 theta3 theta4 theta5], PCI));
        
        %obtenção dos parâmetros de cinemática inversa usando a toolbox
        PCI_Toolbox = robo.ikine(T0_H, [0 0 0 0 0], [0 1 1 1 1 1]);

        %substitui-se os parâmetros obtidos com a toolbox na matriz de
        %transformação para cinemática inversa 
        T0_H_Toolbox_inv = eval(subs(T_0G, [d1 d2 th3 th4 th5], PCI_Toolbox));
        

        if(abs((T0_H_values_inv - T0_H_Toolbox_inv)) < 1e-15)
            disp('Os valores obtidos são coincidentes com os obtidos a partir da toolbox');
        else
            disp('As matrizes não coincidem!');
        end
    end
    
end     %fim do menu
