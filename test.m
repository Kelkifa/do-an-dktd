function varargout = test(varargin)
tic
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @test_OpeningFcn, ...
                   'gui_OutputFcn',  @test_OutputFcn, ...
                   'gui_LayoutFcn',  [], ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
   gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT

function test_OpeningFcn(hObject, eventdata, handles, varargin)

% Choose default command line output for test
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% --- Outputs from this function are returned to the command line.
function varargout = test_OutputFcn(hObject, eventdata, handles)


varargout{1} = handles.output;

% --- Executes on button press in btnStart.
function btnStart_Callback(hObject, eventdata, handles)

button_state = get(hObject,'Value');
 if button_state == get(hObject,'Max')
    cla(handles.pictureBox1);
    clearvars time pPV time_out ;
    
	set(handles.btnStart,'string','Stop')
    set(handles.btnStart,'BackgroundColor','red');

    PV=0;
    output=0;
    error=0;

    time = now;
    startTime = clock;
    
    pSetpoint = 0;
    pPV = 0;  % actual possition (Process Value)
    pInterval = 0;
    pError = 0;   % how much SP and PV are diff (SP - PV)
    pIntegral = 0; % curIntegral + (error * Delta Time)
    pDerivative = 0;  %error - prev error) / Delta time
    preError=0; % error from last time (previous Error)
    pKp = 1;      pKi = 1;    pKd = 1; % PID constant multipliers
    pDt = 0; % delta time
    pOutput = 0; % the drive amount that effects the PV.
    
    pNoisePercent = 0; % amount of the full range to randomly alter the PV
    pNoise = 0;  % random noise added to PV
    noisePercent=0;
    noise=0;
    
    lastderivative=0;
    preIntegral=0;
    ubias = 0;
    minn = -100;
    maxx=100;
    Dt=0;
    lastError=0;
    
    timeInterval = 0.005; %Pause
    filter = 0.05;
    
    lastTime = now;
    ppPV=0;
    ppOutput=0;
    ppsetpoint=0;
    
    kp=0;
    ki=0;
    kd=0;    
    
    plotHandle = plot(handles.pictureBox1,time,ppPV,'Marker','.','LineWidth',.25,'Color',[0 1 0]);
    hold on;
%     grid on;
    plotHandle2 = plot(handles.pictureBox1,time,ppsetpoint,'Marker','.','LineWidth',.25,'Color','red');
    plotHandle3 = plot(handles.pictureBox1,time,ppOutput,'Marker','.','LineWidth',.22,'Color',[0 0 1]);
    xlim(handles.pictureBox1,[max(time-.005) max(time+0.0005)]);
    ylim(handles.pictureBox1,[-10,100]);    
    
    leg = legend('(PV)','(SP)','(OP)');
    legtxt = findobj(leg,'type','text');    

    lastKi=pKi;

    count = 1;
    while (1) %button_state == get(hObject,'Max')        
        pSetpoint =get(handles.trackBarSP,'value');   

        if(get(handles.trackBarSP,'Max') > maxx)
            pSetpoint = maxx;
            set(handles.lblSP,'String',num2str(pSetpoint));
        elseif(get(handles.trackBarSP,'Min') < minn)
            pSetpoint = minn;
            set(handles.lblSP,'String',num2str(pSetpoint));
        end

        pDt = get(handles.trackBarInterval,'value');
        
        if(get(handles.trackBarInterval,'value') > maxx)
            pDt = maxx;
            set(handles.lblInterval,'String',num2str(pDt));
        elseif(get(handles.trackBarInterval,'value') < minn)
            pDt = minn;
            set(handles.lblInterval,'String',num2str(pDt));
        end
% 
        pKp = str2double(get(handles.tbKp,'string'));
        pKi = str2double(get(handles.tbKi,'string'));  
        pKd = str2double(get(handles.tbKd,'string'));
        
        if (pKi~=lastKi)
            % Reset integrator
            pIntegral=0; %errSum = 0;  
            ubias = output - pKp * error; %error=pError
        end        

        if get(handles.cbNoise,'Value') == 1
            pNoise= get(handles.slideNoisePersent,'value');
            set(handles.nudNoisePercent,'String',num2str(pNoise));
        
            r = rand;    
            pNoisePercent = (str2double(get(handles.nudNoisePercent,'string'))) / 100.0;
            noise = (100 * pNoisePercent) * (r - 0.5);
        else
            noise = 0;
        end
        time(count) = datenum(clock);
        x = datenum(clock);
        time_out(count) = etime(clock,startTime);
        ssetpoint(count)= pSetpoint;

%         previous_error = 0 
%         integral = 0 
%         start: 
%         error = setpoint - actual_position 
%         integral = integral + (error*dt) 
%         derivative = (error - previous_error)/dt 
%         output = (Kp*error) + (Ki*integral) + (Kd*derivative) 
% 
%         previous_error = error 
%         wait(dt) 
%         goto start

        Dt = (now - lastTime)*3600*24;  
        pDt=pDt/100;
        lastTime = now;
        error = pSetpoint - PV;
        pIntegral = preIntegral + (error * pDt);
%         dErr = (error - preError) / Dt;
        derivative_raw = (error - preError) / pDt;
%         pDerivative = (error - preError) / pDt;
        pDerivative = filter * derivative_raw + (1 - filter) * lastderivative;
%       
        PV = PV + (output * pDt) -(PV * pDt)+ noise;
%          PV = PV + (output * 0.20) - (PV * 0.10) + noise;
        output = ubias + (pKp * error) + (pKi * pIntegral) + (pKd * pDerivative);     
        
%         s=pIntergral;
%         1/s=pDerivative;
%         Ps=Plant
%         Gs*Hs=Cs*Ps*Hs=output*Ps
%         num=1;
%         den=s*s+10*s+20;
%         
        if isnan(output)
            output = 0;
        end
        if output > maxx
            output = maxx;
        end
        if output < minn
            output = minn;
        end
        if PV > maxx
            PV = maxx;
        end
        if PV < minn
            PV = minn;
        end
        
        % Anti-reset Wind-up
        if (output<=minn),
            pIntegral = pIntegral - (pKi*error * pDt);
        end
        if (output>=maxx),
            pIntegral = pIntegral - (pKi*error * pDt);              
        end

        output = round(output);
        PV=round(PV);
        pSetpoint=round(pSetpoint);
        % Save Variables
        preError = error;
        lastderivative = pDerivative;  
        preIntegral=pIntegral;
        ppOutput(count) = output;   
        ppPV(count)=PV;

%                 Display Line
        disp([
            ' (OP): ' num2str(output,'%3.0f') ...
            ' (PV): ' num2str(PV,'%6.2f') ...
            ' Sp: ' num2str(pSetpoint,'%6.2f')]);

         disp([' DT: ' num2str(Dt,'%6.2f')]);
          
        set(plotHandle,'YData',ppPV,'XData',time);
        set(plotHandle2,'YData',ssetpoint,'XData',time);
        set(plotHandle3,'YData',ppOutput,'XData',time);
        
        set(handles.pictureBox1,'xlim',[max(time)-.00005 max(time)+.00001]);
        
        pNoise= get(handles.slideNoisePersent,'value');
        set(handles.nudNoisePercent,'String',num2str(pNoise));
             
        set(handles.lblError,'string',num2str(error))
        set(handles.lblIntegral,'string',num2str(pIntegral))
        set(handles.lblDeriv,'string',num2str(pDerivative))    
                         
        set(handles.lblOutput,'string',num2str(round(output)))
        set(handles.lblPV,'string',num2str(round(PV)))
                       
        pPV = str2double(get(handles.lblPV,'string')); 
        set(handles.progBarPV,'value',round(pPV))
        pOutput = str2double(get(handles.lblOutput,'string'));
        set(handles.progBarOut,'value',round(pOutput))
                       
        pSetpoint = num2str(get(handles.trackBarSP,'value')); 
        set(handles.lblSP,'string',pSetpoint);
              
        kp = get(handles.tbKp,'string');
        ki = get(handles.tbKi,'string');            
        kd = get(handles.tbKd,'string');
        
        set(handles.tbKp,'string',kp);
        set(handles.tbKi,'string',ki);
        set(handles.tbKd,'string',kd);       
        
        %set(figureHandle,'Visible','on');
        datetick('x','HH:MM:SS','keeplimits');        
        pause(timeInterval);
        count = count +1;
        % save values
        lastKi = pKi; 
        
        drawnow;

%         if get(handles.btnStart,'Userdata')
%              clc            % close if the EXIT button has been pressed
% %              close all
%              clear all
%              % or use break
%             continue
%         end
    end 
    
        set(handles.pictureBox1,'xlim',[min(time)-.01 max(time)+.01]);
        pause(.01);

%         set(handles.btnStart,'String','Start');
%         set(handles.btnStart,'BackgroundColor',[0 .7 0]);
elseif button_state == get(hObject,'Min')
	set(handles.btnStart,'string','Start')
    set(handles.btnStart,'BackgroundColor','green');
    uiwait;

end

% --- Executes during object creation, after setting all properties.
function btnStart_CreateFcn(hObject, eventdata, handles)

function btnStop_Callback(hObject, eventdata, handles)

if ~strcmp(get(handles.btnStart,'String'),'Stop')
    set(handles.btnStart, 'String', 'Start');
    
   clc            % close if the plotting has been stop in advance
else
   set(handles.btnStart,'Userdata',1)
end
 cla(handles.axes1);

function tbKp_Callback(hObject, eventdata, handles)
% hObject    handle to tbKp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

kp = get(handles.tbKp,'string');
set(handles.tbKp,'string',kp)
% --- Executes during object creation, after setting all properties.
function tbKp_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tbKp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function tbKi_Callback(hObject, eventdata, handles)
% hObject    handle to tbKi (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%pKi=0.01;
ki = get(handles.tbKi,'string');
set(handles.tbKi,'string',ki)
% --- Executes during object creation, after setting all properties.
function tbKi_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tbKi (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function tbKd_Callback(hObject, eventdata, handles)
% pKd=1;
kd = get(handles.tbKd,'string');
set(handles.tbKd,'string',kd)

function tbKd_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tbKd (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function cbNoise_Callback(hObject, eventdata, handles)
%
% --- Executes on slider movement.
function trackBarSP_Callback(hObject, eventdata, handles)
% hObject    handle to trackBarSP (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
pSetpoint = get(handles.trackBarSP,'Value');
set(handles.lblSP,'string',num2str(round(pSetpoint)))

function trackBarSP_CreateFcn(hObject, eventdata, handles)
% hObject    handle to trackBarSP (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function trackBarInterval_Callback(hObject, eventdata, handles)
% hObject    handle to trackBarInterval (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
pDt = get(handles.trackBarInterval,'Value');
set(handles.lblInterval,'string',num2str(round(pDt)))

function trackBarInterval_CreateFcn(hObject, eventdata, handles)

if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function progBarPV_Callback(hObject, eventdata, handles)
% hObject    handle to progBarPV (see GCBO)

pPV = round(get(handles.progBarPV,'value'));
set(handles.lblPV,'string',num2str(round(pPV)))
% set(handles.pPV,'string',num2str(round(lblPV)))
% --- Executes during object creation, after setting all properties.
function progBarPV_CreateFcn(hObject, eventdata, handles)
% hObject    handle to progBarPV (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
% --- Executes on slider movement.
function progBarOut_Callback(hObject, eventdata, handles)
% hObject    handle to progBarOut (see GCBO)
pOutput = get(handles.progBarOut,'value');
set(handles.lblOutput,'string',num2str(round(pOutput)))
% set(handles.pPV,'string',num2str(round(lblPV)))
% --- Executes during object creation, after setting all properties.
function progBarOut_CreateFcn(hObject, eventdata, handles)
% hObject    handle to progBarOut (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function slideNoisePersent_Callback(hObject, eventdata, handles)
% hObject    handle to slideNoisePersent (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
            pNoise= get(handles.slideNoisePersent,'value');
            set(handles.nudNoisePercent,'String',num2str(pNoise));
% --- Executes during object creation, after setting all properties.
function slideNoisePersent_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slideNoisePersent (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function nudNoisePercent_Callback(hObject, eventdata, handles)
% hObject    handle to nudNoisePercent (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
            pNoise= get(handles.slideNoisePersent,'value');
            set(handles.nudNoisePercent,'String',num2str(pNoise));
% --- Executes during object creation, after setting all properties.
function nudNoisePercent_CreateFcn(hObject, eventdata, handles)
% hObject    handle to nudNoisePercent (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
% --- Executes during object deletion, before destroying properties.
function pictureBox1_DeleteFcn(hObject, eventdata, handles)
% hObject    handle to pictureBox1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% --- Executes during object creation, after setting all properties.
function lblSP_CreateFcn(hObject, eventdata, handles)
% hObject    handle to lblSP (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% --- Executes on slider movement
% --- Executes during object creation, after setting all properties.
function lblError_CreateFcn(hObject, eventdata, handles)
% hObject    handle to lblError (see GCBO)
% even all CreateFcns called
% --- Executes during object creation, after setting all properties.
function lblIntegral_CreateFcn(hObject, eventdata, handles)
% hObject    handle to lblIntegral (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% --- Executes during object creation, after setting all properties.
function lblDeriv_CreateFcn(hObject, eventdata, handles)
% hObject    handle to lblDeriv (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% --- Executes during object creation, after setting all properties.
function lblPV_CreateFcn(hObject, eventdata, handles)
% hObject    handle to lblPV (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns calle
% --- Executes during object creation, after setting all properties.
function lblOutput_CreateFcn(hObject, eventdata, handles)
% hObject    handle to lblOutput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% --- Executes during object creation, after setting all properties.
function lblInterval_CreateFcn(hObject, eventdata, handles)
% hObject    handle to lblInterval (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% function tmrPV_Tick(hObject, eventdata, handles)
% PV=pPV;
% PV = PV + (output * 0.20) - (PV * 0.10) + noise;
% 
% 
% function tmrPID_Ctrl(hObject, eventdata, handles)
% error = setpoint - PV;  
% % track error over time, scaled to the timer interval
% integral = integral + (error * Dt);
% % determin the amount of change from the last time checked
% derivative = (error - preError) / Dt; 
% 
% % calculate how much drive the output in order to get to the 
% % desired setpoint. 
% output = (Kp * error) + (Ki * integral) + (Kd * derivative);
% 
% % remember the error for the next time around.
% preError = error;    

% --- Executes on button press in btnStop.
