function varargout = bodao(varargin)
% BODAO MATLAB code for bodao.fig
%      BODAO, by itself, creates a new BODAO or raises the existing
%      singleton*.
%
%      H = BODAO returns the handle to a new BODAO or the handle to
%      the existing singleton*.
%
%      BODAO('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in BODAO.M with the given input arguments.
%
%      BODAO('Property','Value',...) creates a new BODAO or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before bodao_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to bodao_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help bodao

% Last Modified by GUIDE v2.5 08-Jul-2013 20:17:37

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @bodao_OpeningFcn, ...
                   'gui_OutputFcn',  @bodao_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
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


% --- Executes just before bodao is made visible.
function bodao_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to bodao (see VARARGIN)

% Choose default command line output for bodao
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes bodao wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = bodao_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes during object creation, after setting all properties.
function bodao_wide_CreateFcn(hObject, eventdata, handles)
% hObject    handle to bodao_wide (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function bodao_narrow_CreateFcn(hObject, eventdata, handles)
% hObject    handle to bodao_narrow (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function bodao_freq_CreateFcn(hObject, eventdata, handles)
% hObject    handle to bodao_freq (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in bodao_set.
function bodao_set_Callback(hObject, eventdata, handles)
% hObject    handle to bodao_set (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
axes(handles.axes1);
cla;
global tt;

m = str2double(get(handles.bodao_m, 'String'));
n = str2double(get(handles.bodao_n, 'String'));
a = str2double(get(handles.bodao_wide, 'String'))/1000;
b = str2double(get(handles.bodao_narrow, 'String'))/1000;
freq = str2double(get(handles.bodao_freq, 'String'))*1000000;
u = 4*pi*10^(-7);
e = 8.854*10^(-12);
kc = sqrt((m*pi/a)^2 + (n*pi/b)^2);
k = 2*pi*freq / (3*10^8);
if k < kc
    errordlg('该工作频率下截止！','错误','on')
else
    B = k * sqrt(1-(kc / k)^2);
    w = B * 3*10^8;
    lg = 2*pi / B;
    t = 0;
    %mesh(x1,y1,z1);
    if get(handles.bodao_TM, 'Value') == 1 %TM
        if m == 0 || n == 0
            errordlg('TM0x、TMx0不存在！','错误','on')
        else
            while(1)
                caiyang = get(handles.bodao_cai, 'Value');
                x = 0:a/caiyang:a;
                y = 0:b/caiyang:b;
                z = 0:lg/caiyang:lg;
                [x1,y1,z1] = meshgrid(x,y,z);
                if get(handles.checkelec, 'Value') == 1
                    Ez = sin(m*pi*x1/a) .* sin(n*pi*y1/b) .* sin(pi/2 - w*t - B*z1);
                    Ex = (B / kc^2)*(m*pi/a) .* sin(pi/2 - m*pi*x1/a) .* sin(n*pi*y1/b) .* sin(w*t - B*z1);
                    Ey = (B / kc^2)*(n*pi/b) .* sin(m*pi*x1/a) .* sin(pi/2 - n*pi*y1/b) .* sin(w*t - B*z1);
                    p1 = quiver3(z1,x1,y1,Ez,Ex,Ey,'b');
                else
                    p1 = quiver3(0,0,0,0,0,0,'b');
                end
                hold on;
                if get(handles.checkmega, 'Value') == 1
                    Hz = zeros(size(z1));
                    Hx = -(w*e/kc^2)*(n*pi/b) .* sin(m*pi*x1/a) .* sin(pi/2 - n*pi*y1/b) .* sin(w*t - B*z1);
                    Hy = (w*e/kc^2)*(m*pi/a) .* sin(pi/2 - m*pi*x1/a) .* sin(n*pi*y1/b) .* sin(w*t - B*z1);
                    p2 = quiver3(z1,x1,y1,Hz,Hx,Hy,'r');
                else
                    p2 = quiver3(0,0,0,0,0,0,'r');
                end
                view(get(handles.bodao_shui, 'Value'), get(handles.bodao_chui, 'Value'));
                xlabel('传输方向/\lambda_g');
                ylabel('波导宽边/m');
                zlabel('波导窄边/m');
                title('矩形波导场分布三维动态演示');
                set(gca,'XTick',-0.5*lg:0.5*lg:1.5*lg,'XTickLabel',{'-0.5','0','0.5','1','1.5'});
                axis([-0.5*lg,1.5*lg,0,a,0,b]);
                legend([p1,p2],'电场分布','磁场分布');
                hold off;
                t = t + 2*pi/w/10;
                pause(get(handles.speedslider, 'Value'));
                if tt == 1
                    tt = 0;
                    break;
                end
            end
        end
    elseif get(handles.bodao_TE, 'Value') == 1 %TE
        if m == 0 && n == 0
            errordlg('TE00不存在！','错误','on')
        else
            while(1)
                caiyang = get(handles.bodao_cai, 'Value');
                x = 0:a/caiyang:a;
                y = 0:b/caiyang:b;
                z = 0:lg/caiyang:lg;
                [x1,y1,z1] = meshgrid(x,y,z);
                if get(handles.checkmega, 'Value') == 1
                    Hz = sin(pi/2 - m*pi*x1/a) .* sin(pi/2 - n*pi*y1/b) .* sin(pi/2 - w*t + B*z1);
                    Hx = -(B / kc^2)*(m*pi/a) .* sin(m*pi*x1/a) .* sin(pi/2 - n*pi*y1/b) .* sin(w*t - B*z1);
                    Hy = -(B / kc^2)*(n*pi/b) .* sin(pi/2 - m*pi*x1/a) .* sin(n*pi*y1/b) .* sin(w*t - B*z1);
                    p1 = quiver3(z1,x1,y1,Hz,Hx,Hy,'b');
                else
                    p1 = quiver3(0,0,0,0,0,0,'b');
                end
                hold on;
                if get(handles.checkelec, 'Value') == 1
                    Ez = zeros(size(z1));
                    Ex = -(w*u/kc^2)*(n*pi/b) .* sin(pi/2 - m*pi*x1/a) .* sin(n*pi*y1/b) .* sin(w*t - B*z1);
                    Ey = (w*u/kc^2)*(m*pi/a) .* sin(m*pi*x1/a) .* sin(pi/2 - n*pi*y1/b) .* sin(w*t - B*z1);
                    p2 = quiver3(z1,x1,y1,Ez,Ex,Ey,'r');
                else
                    p2 = quiver3(0,0,0,0,0,0,'r');
                end
                view(get(handles.bodao_shui, 'Value'), get(handles.bodao_chui, 'Value'));
                xlabel('传输方向/\lambda_g');
                ylabel('波导宽边/m');
                zlabel('波导窄边/m');
                title('矩形波导场分布三维动态演示');
                set(gca,'XTick',-0.5*lg:0.5*lg:1.5*lg,'XTickLabel',{'-0.5','0','0.5','1','1.5'});
                axis([-0.5*lg,1.5*lg,0,a,0,b]);
                legend([p1,p2],'磁场分布','电场分布');
                hold off;
                t = t + 2*pi/w/10;
                pause(get(handles.speedslider, 'Value'));
                if tt == 1
                    tt = 0;
                    break;
                end
            end
        end
    end
end


% --- Executes on button press in bodao_set2D.
function bodao_set2D_Callback(hObject, eventdata, handles)
% hObject    handle to bodao_set2D (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global tt;
tt = 1;

m = str2double(get(handles.bodao_m, 'String'));
n = str2double(get(handles.bodao_n, 'String'));
a = str2double(get(handles.bodao_wide, 'String'))/1000;
b = str2double(get(handles.bodao_narrow, 'String'))/1000;
freq = str2double(get(handles.bodao_freq, 'String'))*1000000;
u = 4*pi*10^(-7);
e = 8.854*10^(-12);
kc = sqrt((m*pi/a)^2 + (n*pi/b)^2);
k = 2*pi*freq / (3*10^8);
if k < kc
    errordlg('该工作频率下截止！','错误','on')
else
    B = k * sqrt(1-(kc / k)^2);
    w = B * 3*10^8;
    lg = 2*pi / B;
    caiyang = get(handles.bodao_cai, 'Value');
    x = 0:a/caiyang:a;
    y = 0:b/caiyang:b;
    z = 0:lg/caiyang:lg;
    if get(handles.bodao_TM, 'Value') == 1 %TM
        if m == 0 || n == 0
            errordlg('TM0x、TMx0不存在！','错误','on')
        else
            if get(handles.bodao_zhu, 'Value') == 1
                [x1,y1] = meshgrid(x,y);
                z1 = 0;
                if get(handles.checkelec2D, 'Value') == 1
                    Ex = (B / kc^2)*(m*pi/a) .* sin(pi/2 - m*pi*x1/a) .* sin(n*pi*y1/b) .* sin(pi/2 - B*z1);
                    Ey = (B / kc^2)*(n*pi/b) .* sin(m*pi*x1/a) .* sin(pi/2 - n*pi*y1/b) .* sin(pi/2 - B*z1);
                    p1 = quiver(x1,y1,Ex,Ey,'b');
                else
                    p1 = quiver(0,0,0,0,'b');
                end
                hold on;
                if get(handles.checkmega2D, 'Value') == 1
                    Hx = -(w*e/kc^2)*(n*pi/b) .* sin(m*pi*x1/a) .* sin(pi/2 - n*pi*y1/b) .* sin(pi/2 - B*z1);
                    Hy = (w*e/kc^2)*(m*pi/a) .* sin(pi/2 - m*pi*x1/a) .* sin(n*pi*y1/b) .* sin(pi/2 - B*z1);
                    p2 = quiver(x1,y1,Hx,Hy,'r');
                else
                    p2 = quiver(0,0,0,0,'r');
                end
                xlabel('波导宽边/m');
                ylabel('波导窄边/m');
                title('矩形波导场分布二维截面图');
                axis([0,a,0,b]);
                legend([p1,p2],'电场分布','磁场分布');
                hold off;
            elseif get(handles.bodao_ce, 'Value') == 1
                [y1,z1] = meshgrid(y,z);
                x1 = a/(2*m);
                if get(handles.checkelec2D, 'Value') == 1
                    Ez = sin(m*pi*x1/a) .* sin(n*pi*y1/b) .* sin(B*z1);
                    Ey = (B / kc^2)*(n*pi/b) .* sin(m*pi*x1/a) .* sin(pi/2 - n*pi*y1/b) .* sin(pi/2 - B*z1);
                    p1 = quiver(z1,y1,Ez,Ey,'b');
                else
                    p1 = quiver(0,0,0,0,'b');
                end
                hold on;
                if get(handles.checkmega, 'Value') == 1
                    %if mod(m,2) == 1
                        Hx = -(w*e/kc^2)*(n*pi/b) .* sin(m*pi*x1/a) .* sin(pi/2 - n*pi*y1/b) .* sin(pi/2 - B*z1);
                        ss = size(Hx);
                        maxx = max(max(abs(Hx))) / 2;
                        for i = 1:ss(1)
                            for j = 1:ss(2)
                                if Hx(i,j) > maxx
                                    p2 = plot(z1(i,2),y1(2,j),'r-*','linewidth',3,'Markersize',3);
                                    hold on;
                                elseif Hx(i,j) < 0-maxx
                                    p2 = plot(z1(i,2),y1(2,j),'+r');
                                    hold on;
                                end
                            end
                        end
                    %else
                        %Hz = zeros(size(z1));
                        %Hy = (w*e/kc^2)*(m*pi/a) .* sin(pi/2 - m*pi*x1/a) .* sin(n*pi*y1/b) .* sin(pi/2 - B*z1);
                        %p2 = quiver(z1,y1,Hz,Hy,'r');
                    %end
                else
                    p2 = quiver(0,0,0,0,'r');
                end
                xlabel('传输方向/\lambda_g');
                ylabel('波导窄边/m');
                title('矩形波导场分布二维截面图');
                set(gca,'XTick',-0.5*lg:0.5*lg:1.5*lg,'XTickLabel',{'-0.5','0','0.5','1','1.5'});
                axis([-0.5*lg,1.5*lg,0,b]);
                legend([p1,p2],'电场分布','磁场分布');
                hold off;
            elseif get(handles.bodao_fu, 'Value') == 1
                [x1,z1] = meshgrid(x,z);
                y1 = b/(2*n);
                if get(handles.checkelec2D, 'Value') == 1
                    Ez = sin(m*pi*x1/a) .* sin(n*pi*y1/b) .* sin(B*z1);
                    Ex = (B / kc^2)*(m*pi/a) .* sin(pi/2 - m*pi*x1/a) .* sin(n*pi*y1/b) .* sin(pi/2 - B*z1);
                    p1 = quiver(z1,x1,Ez,Ex,'b');
                else
                    p1 = quiver(0,0,0,0,'b');
                end
                hold on;
                if get(handles.checkmega2D, 'Value') == 1
                        Hy = (w*e/kc^2)*(m*pi/a) .* sin(pi/2 - m*pi*x1/a) .* sin(n*pi*y1/b) .* sin(pi/2 - B*z1);
                        ss = size(Hy);
                        maxx = max(max(abs(Hy))) / 2;
                        for i = 1:ss(1)
                            for j = 1:ss(2)
                                if Hy(i,j) > maxx
                                    p2 = plot(z1(i,2),x1(2,j),'r-*','linewidth',3,'Markersize',3);
                                    hold on;
                                elseif Hy(i,j) < 0-maxx
                                    p2 = plot(z1(i,2),x1(2,j),'+r');
                                    hold on;
                                end
                            end
                        end
                    %Hz = zeros(size(z1));
                    %Hx = -(w*e/kc^2)*(n*pi/b) .* sin(m*pi*x1/a) .* sin(pi/2 - n*pi*y1/b) .* sin(pi/2 - B*z1);
                    %p2 = quiver(z1,x1,Hz,Hx,'r');
                else
                    p2 = quiver(0,0,0,0,'r');
                end
                xlabel('传输方向/\lambda_g');
                ylabel('波导宽边/m');
                title('矩形波导场分布二维截面图');
                set(gca,'XTick',-0.5*lg:0.5*lg:1.5*lg,'XTickLabel',{'-0.5','0','0.5','1','1.5'});
                axis([-0.5*lg,1.5*lg,0,a]);
                legend([p1,p2],'电场分布','磁场分布');
                hold off;
            end;
        end
    elseif get(handles.bodao_TE, 'Value') == 1 %TE
        if m == 0 && n == 0
            errordlg('TE00不存在！','错误','on')
        else
            if get(handles.bodao_zhu, 'Value') == 1
                [x1,y1] = meshgrid(x,y);
                z1 = 0;
                if get(handles.checkmega2D, 'Value') == 1
                    Hx = -(B / kc^2)*(m*pi/a) .* sin(m*pi*x1/a) .* sin(pi/2 - n*pi*y1/b) .* sin(pi/2 - B*z1);
                    Hy = -(B / kc^2)*(n*pi/b) .* sin(pi/2 - m*pi*x1/a) .* sin(n*pi*y1/b) .* sin(pi/2 - B*z1);
                    p1 = quiver(x1,y1,Hx,Hy,'b');
                else
                    p1 = quiver(0,0,0,0,'b');
                end
                hold on;
                if get(handles.checkelec2D, 'Value') == 1
                    Ex = -(w*u/kc^2)*(n*pi/b) .* sin(pi/2 - m*pi*x1/a) .* sin(n*pi*y1/b) .* sin(pi/2 - B*z1);
                    Ey = (w*u/kc^2)*(m*pi/a) .* sin(m*pi*x1/a) .* sin(pi/2 - n*pi*y1/b) .* sin(pi/2 - B*z1);
                    p2 = quiver(x1,y1,Ex,Ey,'r');
                else
                    p2 = quiver(0,0,0,0,'r');
                end
                xlabel('波导宽边/m');
                ylabel('波导窄边/m');
                title('矩形波导场分布二维截面图');
                axis([0,a,0,b]);
                legend([p1,p2],'磁场分布','电场分布');
                hold off;
            elseif get(handles.bodao_ce, 'Value') == 1
                [y1,z1] = meshgrid(y,z);
                x1 = 0;
                if get(handles.checkmega2D, 'Value') == 1
                    Hz = sin(pi/2 - m*pi*x1/a) .* sin(pi/2 - n*pi*y1/b) .* sin(B*z1);
                    Hy = -(B / kc^2)*(n*pi/b) .* sin(pi/2 - m*pi*x1/a) .* sin(n*pi*y1/b) .* sin(pi/2 - B*z1);
                    p1 = quiver(z1,y1,Hz,Hy,'b');
                else
                    p1 = quiver(0,0,0,0,'b');
                end
                hold on;
                if get(handles.checkelec2D, 'Value') == 1
                    if n ~= 0
                        Ex = -(w*u/kc^2)*(n*pi/b) .* sin(pi/2 - m*pi*x1/a) .* sin(n*pi*y1/b) .* sin(pi/2 - B*z1);
                        ss = size(Ex);
                        maxx = max(max(abs(Ex))) / 2;
                        for i = 1:ss(1)
                            for j = 1:ss(2)
                                if Ex(i,j) > maxx
                                    p2 = plot(z1(i,2),y1(2,j),'r-*','linewidth',3,'Markersize',3);
                                    hold on;
                                elseif Ex(i,j) < 0-maxx
                                    p2 = plot(z1(i,2),y1(2,j),'+r');
                                    hold on;
                                end
                            end
                        end
                    elseif n == 0
                        Ez = zeros(size(z1));
                        Ey = (w*u/kc^2)*(m*pi/a) .* sin(pi/2 - n*pi*y1/b) .* sin(pi/2 - B*z1);
                        p2 = quiver(z1,y1,Ez,Ey,'r');
                    end
                else
                    p2 = quiver(0,0,0,0,'r');
                end
                xlabel('传输方向/\lambda_g');
                ylabel('波导窄边/m');
                title('矩形波导场分布二维截面图');
                set(gca,'XTick',-0.5*lg:0.5*lg:1.5*lg,'XTickLabel',{'-0.5','0','0.5','1','1.5'});
                axis([-0.5*lg,1.5*lg,0,b]);
                legend([p1,p2],'磁场分布','电场分布');
                hold off;
            elseif get(handles.bodao_fu, 'Value') == 1
                [x1,z1] = meshgrid(x,z);
                y1 = 0;
                if get(handles.checkmega2D, 'Value') == 1
                    Hz = sin(pi/2 - m*pi*x1/a) .* sin(pi/2 - n*pi*y1/b) .* sin(B*z1);
                    Hx = -(B / kc^2)*(m*pi/a) .* sin(m*pi*x1/a) .* sin(pi/2 - n*pi*y1/b) .* sin(pi/2 - B*z1);
                    p1 = quiver(z1,x1,Hz,Hx,'b');
                else
                    p1 = quiver(0,0,0,0,'b');
                end
                hold on;
                if get(handles.checkelec2D, 'Value') == 1
                    %if n ~= 0
                        Ey = (w*u/kc^2)*(m*pi/a) .* sin(m*pi*x1/a) .* sin(pi/2 - n*pi*y1/b) .* sin(pi/2 - B*z1);
                        ss = size(Ey);
                        maxx = max(max(abs(Ey))) / 2;
                        for i = 1:ss(1)
                            for j = 1:ss(2)
                                if Ey(i,j) > maxx
                                    p2 = plot(z1(i,2),x1(2,j),'r-*','linewidth',3,'Markersize',3);
                                    hold on;
                                elseif Ey(i,j) < 0-maxx
                                    p2 = plot(z1(i,2),x1(2,j),'+r');
                                    hold on;
                                end
                            end
                        end
                    %elseif n == 0
                        %Ez = zeros(size(z1));
                        %Ex = -(w*u/kc^2)*(n*pi/b) .* sin(pi/2 - m*pi*x1/a) .* sin(pi/2 - B*z1);
                        %p2 = quiver(z1,x1,Ez,Ex,'r');
                    %end
                else
                    p2 = quiver(0,0,0,0,'r');
                end
                xlabel('传输方向/\lambda_g');
                ylabel('波导宽边/m');
                title('矩形波导场分布二维截面图');
                set(gca,'XTick',-0.5*lg:0.5*lg:1.5*lg,'XTickLabel',{'-0.5','0','0.5','1','1.5'});
                axis([-0.5*lg,1.5*lg,0,a]);
                legend([p1,p2],'磁场分布','电场分布');
                hold off;
            end
        end
    end
end


% --- Executes on button press in bodao_clear.
function bodao_clear_Callback(hObject, eventdata, handles)
% hObject    handle to bodao_clear (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
plot(0,0)
global tt;
tt = 1;
set(handles.bodao_cai, 'Value', 10);
set(handles.speedslider, 'Value', 0.5);
set(handles.bodao_shui, 'Value', -37.5);
set(handles.bodao_chui, 'Value', 60);


% --- Executes during object creation, after setting all properties.
function bodao_m_CreateFcn(hObject, eventdata, handles)
% hObject    handle to bodao_m (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function bodao_n_CreateFcn(hObject, eventdata, handles)
% hObject    handle to bodao_n (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function angle_CreateFcn(hObject, eventdata, handles)
% hObject    handle to angle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

set(hObject, 'String', {'全方位', '主视图', '侧视图', '俯视图'});


% --- Executes during object creation, after setting all properties.
function speedslider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to speedslider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes during object creation, after setting all properties.
function bodao_shui_CreateFcn(hObject, eventdata, handles)
% hObject    handle to bodao_shui (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes during object creation, after setting all properties.
function bodao_chui_CreateFcn(hObject, eventdata, handles)
% hObject    handle to bodao_chui (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on selection change in angle.
function angle_Callback(hObject, eventdata, handles)
% hObject    handle to angle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns angle contents as cell array
%        contents{get(hObject,'Value')} returns selected item from angle

                switch get(handles.angle, 'Value');
                case 1
                    set(handles.bodao_shui, 'Value', -37.5);
                    set(handles.bodao_chui, 'Value', 60);
                case 2
                    set(handles.bodao_shui, 'Value', 90);
                    set(handles.bodao_chui, 'Value', 0);
                case 3
                    set(handles.bodao_shui, 'Value', 0);
                    set(handles.bodao_chui, 'Value', 0);
                case 4
                    set(handles.bodao_shui, 'Value', 0);
                    set(handles.bodao_chui, 'Value', 90);
                end


% --- Executes on button press in checkmega2D.
function checkmega2D_Callback(hObject, eventdata, handles)
% hObject    handle to checkmega2D (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hint: get(hObject,'Value') returns toggle state of checkmega2D
set(handles.checkmega, 'Value', get(handles.checkmega2D, 'Value'));


% --- Executes on button press in checkelec2D.
function checkelec2D_Callback(hObject, eventdata, handles)
% hObject    handle to checkelec2D (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hint: get(hObject,'Value') returns toggle state of checkelec2D
set(handles.checkelec, 'Value', get(handles.checkelec2D, 'Value'));


% --- Executes on button press in checkmega.
function checkmega_Callback(hObject, eventdata, handles)
% hObject    handle to checkmega (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hint: get(hObject,'Value') returns toggle state of checkmega
set(handles.checkmega2D, 'Value', get(handles.checkmega, 'Value'));


% --- Executes on button press in checkelec.
function checkelec_Callback(hObject, eventdata, handles)
% hObject    handle to checkelec (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hint: get(hObject,'Value') returns toggle state of checkelec
set(handles.checkelec2D, 'Value', get(handles.checkelec, 'Value'));


% --- Executes during object creation, after setting all properties.
function bodao_cai_CreateFcn(hObject, eventdata, handles)
% hObject    handle to bodao_cai (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
