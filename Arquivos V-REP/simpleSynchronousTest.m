% Copyright 2006-2016 Coppelia Robotics GmbH. All rights reserved. 
% marc@coppeliarobotics.com
% www.coppeliarobotics.com
% 
% -------------------------------------------------------------------
% THIS FILE IS DISTRIBUTED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
% WARRANTY. THE USER WILL USE IT AT HIS/HER OWN RISK. THE ORIGINAL
% AUTHORS AND COPPELIA ROBOTICS GMBH WILL NOT BE LIABLE FOR DATA LOSS,
% DAMAGES, LOSS OF PROFITS OR ANY OTHER KIND OF LOSS WHILE USING OR
% MISUSING THIS SOFTWARE.
% 
% You are free to use/modify/distribute this file for whatever purpose!
% -------------------------------------------------------------------
%
% This file was automatically created for V-REP release V3.3.2 on August 29th 2016

% This small example illustrates how to use the remote API
% synchronous mode. The synchronous mode needs to be
% pre-enabled on the server side. You would do this by
% starting the server (e.g. in a child script) with:
%
% simExtRemoteApiStart(19999,1300,false,true)
%
% But in this example we try to connect on port
% 19997 where there should be a continuous remote API
% server service already running and pre-enabled for
% synchronous mode.
%
% IMPORTANT: for each successful call to simxStart, there
% should be a corresponding call to simxFinish at the end!

    clear graph1 graph2 graph3 graph4 graph5 t Yvrep;
    disp('Program started');
    vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    vrep.simxFinish(-1); % just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);
    
    time_step = 0.001;
    time_step2 = 0.0005;
    steps = 6200;
    pict = 1;

    if (clientID>-1)
        disp('Connected to remote API server');

        % enable the synchronous mode on the client:
        vrep.simxSynchronous(clientID,true);
        
        vrep.simxSetFloatingParameter(clientID, vrep.sim_floatparam_simulation_time_step, time_step, vrep.simx_opmode_oneshot_wait);

        % start the simulation:
        vrep.simxStartSimulation(clientID,vrep.simx_opmode_blocking);
        
        % Código executado entre cada passo de simulação
                
        [returnCode,handlejunta1]  = vrep.simxGetObjectHandle(clientID,'junta1',   vrep.simx_opmode_blocking);
        [returnCode,handlejunta2]  = vrep.simxGetObjectHandle(clientID,'junta2',   vrep.simx_opmode_blocking);
        [returnCode,handlebarra1]  = vrep.simxGetObjectHandle(clientID,'pendulo1_din', vrep.simx_opmode_blocking);
        [returnCode,handlebarra2]  = vrep.simxGetObjectHandle(clientID,'pendulo2_din', vrep.simx_opmode_blocking);
        
        %Control Gain
        K = [-61.7835  -28.9532   -9.9639   -5.0677];
        
        last_pos1 = 0;
        curr_pos1 = 0;
        last_pos2 = -2*pi;
        curr_pos2 = 0;
        
        LQR = 0;
        steps_swing = 0;
        
        % Now step a few times:
        for i=0:steps
            returnCode = 1;
            while(returnCode == 1)
                [returnCode,positionjunta1] = vrep.simxGetJointPosition(clientID,handlejunta1,vrep.simx_opmode_blocking);
                curr_pos1 = positionjunta1;
            end
               
            returnCode = 1;
            while(returnCode == 1)
                [returnCode,positionjunta2] = vrep.simxGetJointPosition(clientID,handlejunta2,vrep.simx_opmode_blocking);
                curr_pos2 = positionjunta2;
            end
               
            returnCode = 1;
            while(returnCode == 1)
                [returnCode,lvelocity1, avelocity1] = vrep.simxGetObjectVelocity(clientID,handlebarra1,vrep.simx_opmode_blocking);
            end
               
            returnCode = 1;
            while(returnCode == 1)
                [returnCode,lvelocity2, avelocity2] = vrep.simxGetObjectVelocity(clientID,handlebarra2,vrep.simx_opmode_blocking);
            end
            
           %positionjunta1 = positionjunta1 - pi;
           %curr_pos1 = curr_pos1 - pi;
           
           % Cálculo da velocidade angular - barra 1
           vel1 = (curr_pos1 - last_pos1)/time_step;
           last_pos1 = curr_pos1;
           % Cálculo da velocidade angular - barra 2
           vel2 = (curr_pos2 - last_pos2)/time_step;
           last_pos2 = curr_pos2;
           % Vetor de estados
           x = [positionjunta1 positionjunta2 vel1 vel2]';

           graph1(i+1) = x(1);
           graph2(i+1) = x(2);
           graph3(i+1) = x(3);
           graph4(i+1) = x(4);

           % Troca de controlador
           if(LQR == 0 && (x(1) > 2.8 || x(1) < -2.8) && (x(2) < -0.42 || x(2) > 0.42))
               steps_swing = i;
               % Ativar LQR
               LQR = 1;
               % Alterar time_step para 0.0005 (dividir por 2)
               vrep.simxSetIntegerParameter(clientID,vrep.sim_intparam_speedmodifier, -1, vrep.simx_opmode_oneshot_wait);
           end
           
           % Controlador LQR
           if(LQR == 1)
                % Mudança de variável para linealização
                x(1) = x(1)-pi;
                % Torque
                torque = -K*x;
                % Saturação
                torque = max(min(torque,1.2),-1.2);
                
           % Controlador swing-up
           else
                % Constantes
                Kp = 98.1; Kd = 1.5;
                % Torque
                torque = (Kp*(0.505*atan(x(3)) - x(2)) - Kd*x(4));
                % Saturação
                torque = max(min(torque,0.065),-0.065);
           end
                    
           if(torque > 0)
               returnCode = 1;
               while(returnCode == 1)
                [returnCode]= vrep.simxSetJointTargetVelocity(clientID, handlejunta2,99999,vrep.simx_opmode_oneshot);
               end
               
               returnCode = 1;
               while(returnCode == 1)
                [returnCode]= vrep.simxSetJointForce(clientID,handlejunta2,torque,vrep.simx_opmode_oneshot);
               end
           else
               returnCode = 1;
               while(returnCode == 1)
                [returnCode]= vrep.simxSetJointTargetVelocity(clientID, handlejunta2,-99999,vrep.simx_opmode_oneshot);
               end
               
               returnCode = 1;
               while(returnCode == 1)
                [returnCode]= vrep.simxSetJointForce(clientID,handlejunta2,-torque,vrep.simx_opmode_oneshot);
               end
           end
           
           graph5(i+1) = torque;
           
           if((LQR == 0 && mod(i,21) == 0) || (LQR == 1 && mod(i,42) == 0))
               img = screencapture(0, 'Position', [47 85 1320 600]);
                if(pict < 10)
                    s1 = 'image000';
                elseif(pict < 100)
                    s1 = 'image00';
                elseif(pict < 1000)
                    s1 = 'image0';
                else
                    s1 = 'image';
                end        
                s2 = int2str(pict);
                s3 = '.jpg';
                s = strcat(s1,s2,s3);
                imwrite(img,s);
                pict = pict + 1;
           end
                       
           vrep.simxSynchronousTrigger(clientID);
        end

        figure;
        t1 = 0:time_step:(steps_swing-1)*time_step;
        t2 = ((steps_swing-1)*time_step + time_step/2):time_step/2:((steps_swing-1)*time_step + (steps - steps_swing + 1)*time_step/2);
        t = horzcat(t1,t2);
        SP = ((steps_swing-1)*time_step + time_step/2);
        plot(t,graph1);
        hold on;
        plot(t,graph2);
        hold on;
        %plot(t,graph3);
        %hold on;
        %plot(t,graph4);
        %hold on;
        plot(t,graph5);
        axis([0 ((steps_swing-1)*time_step + (steps - steps_swing + 1)*time_step/2) -110 55]);
        line([SP SP], [-110 55], 'Color', [0 0 0])
        
        legend('theta 1','theta 2','velocidade angular 1','velocidade angular 2','torque');
        figure;
        plot(t,graph5);
        axis([0 ((steps_swing-1)*time_step + (steps - steps_swing + 1)*time_step/2) -1.5 1.5]);
               
        % stop the simulation:
        vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking);

        % Now close the connection to V-REP:    
        vrep.simxFinish(clientID);
        
    else
        disp('Failed connecting to remote API server');
    end
    vrep.delete(); % call the destructor!
    Yvrep = [graph1' graph2' graph3' graph4'];
    
    disp('Program ended');
