function varargout = smith(varargin)

% SMITH M-file for smith.fig
%      SMITH, by itself, creates a new SMITH or raises the existing singleton*.
%      H = SMITH returns the handle to a new SMITH or the handle to the existing singleton*.
%      SMITH('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SMITH.M with the given input arguments.
%      SMITH('Property','Value',...) creates a new SMITH or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before smith_OpeningFunction gets called.  An
%      unrecognized property name or invalid value makes property
%      application  
%   stop.All inputs are passed to smith_OpeningFcn via varargin.
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
% See also: GUIDE, GUIDATA, GUIHANDLES
% Edit the above text to modify the response to help smith
% Last Modified by GUIDE v2.5 10-Jul-2013 14:45:14
% Begin initialization code - DO NOT EDIT

gui_Singleton = 1;
gui_State = struct('gui_Name',  mfilename, ...
                            'gui_Singleton',  gui_Singleton, ...
                            'gui_OpeningFcn', @smith_OpeningFcn, ...
                            'gui_OutputFcn',  @smith_OutputFcn, ...
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

% --- Executes just before smith is made visible.
function smith_OpeningFcn(hObject, eventdata, handles, varargin)      %Smith圆图全局设定
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to smith (see VARARGIN)
% Choose default command line output for smith
global pic1 pic2 edit1 input text5 text6 flag1 flag0;
handles.output = hObject;
edit1=handles.edit1;
text5=handles.text5;
text6=handles.text6;
flag0=1;
flag1=1;
pic1=handles.wave;
pic2=handles.smith;
handles.current_d2=0.125;
handles.current_type = 'u';
input = 'z';
%background
set(gcf,'WindowButtonMotionFcn',@draw)
set(gcf,'WindowButtonDownFcn',@button)

% Update handles structure

guidata(hObject, handles);

% UIWAIT makes smith wait for user response (see UIRESUME) UIWAIT让Smith等待用户响应
% uiwait(handles.figure1);
% --- Outputs from this function are returned to the command line.     从这个函数输出回到命令行

function varargout = smith_OutputFcn(hObject, eventdata, handles) 

% varargout cell array for returning output args (see VARARGOUT);
% hObject handle to figure
% eventdata reserved - to be defined in a future version of MATLAB
% handles structure with handles and user data (see GUIDATA)
% Get default command line output from handles structure

varargout{1} = handles.output;
ssize=get(0,'screensize');
set(hObject,'position',ssize);

function wave_u(zl)        %函数wave_u画出电压图像
global pic1;
aida = (zl-1)/(zl+1);
abs_a = abs(aida);
angle_a = angle(aida);
x = 0:0.01:2;
u = sqrt(1+abs_a^2+2*abs_a*cos(4*pi*x-angle_a));
subplot(pic1);
plot(x, u);
grid on
set(gca,'XDir')
set(gca,'YAxisLocation','left')

function wave_i(zl)         %函数wave_i画出电流图像
global pic1;
aida = (zl-1)/(zl+1);
abs_a = abs(aida);
angle_a = angle(aida);
x = 0:0.01:2;
i = sqrt(1+abs_a^2-2*abs_a*cos(4*pi*x-angle_a));
subplot(pic1);
plot(x,i);
grid on
set(gca,'XDir')
set(gca,'YAxisLocation','left')

function wave_z(zl)            %函数WAVE_Z画出阻抗图像
global pic1;
x = 0:0.01:2;
z = (zl+1i*tan(2*pi*x))./(1+1i*zl*tan(2*pi*x));
real_z = real(z);
imag_z = imag(z);
subplot(pic1);
p1 = plot(x,real_z,'r');
hold on;
p2 = plot(x,imag_z,'g');
grid on;
set(gca,'XDir');
set(gca,'YAxisLocation','left');
legend([p1,p2],'阻抗实部','阻抗虚部');
hold off



%—————————画图 Smith背景图—————————%

function circle(x0,y0,r,color,linetype)       %Smith圆图画圆
t=0:0.01:2*pi;
x=x0+cos(t)*r;
y=y0+sin(t)*r;
plot(x,y,'-','color',color,'HitTest','off');


function arch_x(x0,color)         %画电抗圆
if x0>0
    if x0>=1
         theta=pi-asin(2*x0/(x0^2+1));
         else
         theta=asin(2*x0/(x0^2+1));
    end;
    t=0:0.01:theta;
    x=1-sin(t)*abs(1/x0);
    y=1/x0-cos(t)*abs(1/x0);
     plot(x,y,'-','LineWidth',1,'color',color);
elseif x0<0
    if x0<-1
          theta=pi+asin(2*x0/(x0^2+1));
    else
         theta=-asin(2*x0/(x0^2+1));
    end;
        t=0:0.01:theta;
        x=1-sin(t)*abs(1/x0);
        y=1/x0+cos(t)*abs(1/x0);
        plot(x,y,'-','LineWidth',1,'color',color);
end

function background    %Smith背景图
global pic2;
subplot(pic2);
axis([-1,1,-1,1]);
axis off;
box off;
circle(0,0,1,'k','-'); %画反射系数圆
hold on;       %当前轴及图形保持而不被刷新，准备接受此后将绘制
circle(0,0,0.75,'k','--');
hold on;
circle(0,0,0.5,'k','--');
hold on;
circle(0,0,0.25,'k','--');
hold on;
circle(0,0,0,'k','--');
hold on;
circle(2/3,0,1/3,[0.3,0.4,0.3],'-'); %画阻抗圆
hold on;
circle(1/2,0,1/2,[0.3,0.4,0.3],'-');
hold on;
circle(1/3,0,2/3,[0.3,0.4,0.3],'-');
hold on;
circle(1/5,0,4/5,[0.3,0.4,0.3],'-');
hold on;
arch_x(4,[0.3,0.4,0.3]);  %画电抗圆
hold on;
arch_x(-4,[0.3,0.4,0.3]);
hold on;
arch_x(2,[0.3,0.4,0.3]);
hold on;
arch_x(-2,[0.3,0.4,0.3]);
hold on;
arch_x(1,[0.3,0.4,0.3]);
hold on;
arch_x(-1,[0.3,0.4,0.3]);
hold on;
arch_x(0.5,[0.3,0.4,0.3]);
hold on;
arch_x(-0.5,[0.3,0.4,0.3]);
hold on;
text(1.05*cos(pi/7),1.05*sin(pi/7),'x=4');  %在每个电抗圆上写值
text(1.05*cos(pi/7),-1.05*sin(pi/7),'x=-4');
text(1.05*cos(pi/2),1.05*sin(pi/2),'x=1');
text(-1.05*cos(pi/2),-1.05*sin(pi/2),'x=-1')
text(1.05*cos(pi/3.5),1.05*sin(pi/3.5),'x=2');
text(1.05*cos(pi/3.5),-1.05*sin(pi/3.5),'x=-2');
text(-0.78,0.83,'x=0.5');
text(-0.78,-0.83,'x=0.5');
hold on;
axis off;


function d = distance(zl, z)   %函数distance返回输入阻抗为z的点距负载zl的距离
if(z==Inf)
    d=0.25;
else
    d = real(atan((z - zl)/1i/(1-z*zl))/2/pi);
end
if (d<0)
    d = d+0.5;
end

function single_stub_match(r,x)   %单支节调配
global pic2;
subplot(pic2);
background;
hold on;
gamma=abs((r+x*1i-1)/(r+x*1i+1));  
circle(0,0,gamma,'b','-');
pause(0.5);
circle(0,0,gamma,'w','-');
pause(0.5);
circle(0,0,gamma,'b','-');
pause(0.5);
circle(0,0,gamma,'w','-');
pause(0.5);
circle(0,0,gamma,'b','-');
pause(1);
circle(0.5,0,0.5,'r','-');
pause(0.5);
circle(0.5,0,0.5,'w','-');
pause(0.5);
circle(0.5,0,0.5,'r','-');
pause(0.5);
circle(0.5,0,0.5,'w','-');
pause(0.5);
circle(0.5,0,0.5,'r','-');
plot(gamma^2,sqrt(gamma^2-gamma^4),'kx', 'MarkerSize',10,'LineWidth',2);
plot(gamma^2,-sqrt(gamma^2-gamma^4),'kx','MarkerSize',10,'LineWidth',2);

function edit1_Callback(hObject, eventdata, handles)       %输入归一化阻抗、导纳
% hObject handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of edit1 as text
% str2double(get(hObject,'String')) returns contents of edit1 as a double
global input;
if input == 'z'
    zl = str2double(get(hObject,'String'));
    if isnan(zl)
        errordlg('You must input a number.');
        set(handles.edit1, 'string', '');
    elseif real(zl<0)
        errordlg('The real part cannot be negative.');
        set(handles.edit1, 'string', '');
    else
        set(handles.edit1, 'string', num2str(zl));
    end
else
    zl = 1/str2double(get(hObject,'String'));
    if isnan(zl)
        errordlg('You must input a number.');
        set(handles.edit1, 'string', '');
    elseif real(zl<0)
        errordlg('The real part cannot be negative.');
        set(handles.edit1, 'string', '');
    else
        set(handles.edit1, 'string', num2str(zl));
    end   
end

function button(hObject, eventdata, handles)  %使能按钮
global flag1;
if flag1 == 1
    flag1 = 0;
else
    flag1 = 1;
end;

%实时电阻电抗反射系数图显示
function draw(hObject, eventdata, handles)
global pic2 edit1 input text5 text6 flag1 flag0;
if flag0==1
if flag1 ==1
p = get(gca,'CurrentPoint');  %得到在图上选定的点
x = p(1);  %得到实部，即电阻
y = p(3);  %得到虚部，即电抗
if x^2+y^2 < 1   %判定选定的点是否正确
background;
hold on;
axis([-1,1,-1,1]);
aida = x+y*1i;   %得到选定的点所在的反射系数圆
angle_a = angle(aida);  %反射系数圆的相角
if angle_a<0
    angle_a = angle_a+2*pi;
end;
zmax = angle_a/4/pi;
if angle_a<pi
    zmin = (angle_a+pi)/4/pi;
else
    zmin = (angle_a-pi)/4/pi;
end;
set(text5,'string',zmax);
set(text6,'string',zmin);
r1 = abs(aida); %反射系数的模，反射系数圆的半径
zl = (1+aida)/(1-aida);  
if(input == 'z')
    set(edit1,'string',num2str(zl));
else
    set(edit1,'string',num2str(1/zl));
end
r2 = 1/(1+real(zl));%电阻圆的半径
plot(x,y,'x');
circle(0,0,r1,'r','-');  %画选定点的反射系数圆
circle(real(zl)/(1+real(zl)),0,r2,'g','-');  %画选定点的电阻圆
arch_x(imag(zl),'b');  %画选定点的电抗圆
hold off;
end
end
end

% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
% See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)    %波形选择按钮
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
zl = str2double(get(handles.edit1,'string'));
switch handles.current_type;
case 'u'
    wave_u(zl)
case 'i'
    wave_i(zl)
case 'z'
	wave_z(zl)
end;
background
guidata(hObject,handles)

% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)    %波形选择
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: contents = get(hObject,'String') returns popupmenu1 contents as cell array
% contents{get(hObject,'Value')} returns selected item from popupmenu1
val = get(hObject,'Value');
str = get(hObject, 'String');
zl = str2double(get(handles.edit1,'string'));
switch str{val};
case '电压波形'
    handles.current_type = 'u';
case '电流波形'
    handles.current_type = 'i';
case '阻抗波形'
    handles.current_type = 'z';

end
guidata(hObject,handles)
% --- Executes during object creation, after setting all properties.
function popupmenu1_CreateFcn(hObject, eventdata, handles)
% hObject handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: popupmenu controls usually have a white background on Windows.
% See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function text5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to text5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% --- Executes during object creation, after setting all properties.
function text6_CreateFcn(hObject, eventdata, handles)
% hObject handle to text6 (see GCBO)
% eventdata reserved - to be defined in a future version of MATLAB
% handles empty - handles not created until after all CreateFcns called

% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)   %双支节
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.pushbutton3,'Enable','off');
set(handles.pushbutton10,'Enable','off');
set(gcf,'WindowButtonMotionFcn','');
zl = str2double(get(handles.edit1,'string')); %双支节调配
aida = (zl-1)/(zl+1);
abs_a = abs(aida);
b = 2*abs_a/sqrt(1-abs_a^2);
L1 = distance(0,1i*1/b);
L2 = distance(0,-1i*1/b);
d1 = distance(zl,1/(1+1i*b));
d2 = distance(zl,1/(1-1i*b));
set(handles.d1,'string',num2str(d1))
set(handles.d2,'string',num2str(d2))
set(handles.L1,'string',num2str(L1))
set(handles.L2,'string',num2str(L2))
guidata(hObject,handles)
r = real(zl);
x = imag(zl);
single_stub_match(r,x) %单支节
set(handles.pushbutton3,'Enable','on');
set(handles.pushbutton10,'Enable','on');
set(gcf,'WindowButtonMotionFcn',@draw);
hold off
guidata(hObject,handles);

function dis1_Callback(hObject, eventdata, handles)
% hObject    handle to dis (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of dis as text
%        str2double(get(hObject,'String')) returns contents of dis as a
%        double
global flag0;
d1 = str2double(get(hObject,'String'));
if isnan(d1) %双支节部分的输入
    flag0=0;
    errordlg('You must input a number.');
    set(handles.dis1, 'string', '');
elseif (d1<0)
    flag0=0;
    errordlg('Distance cannot be negative.');
    set(handles.dis1, 'string', '');
else
    handles.current_d1 = d1;
    guidata(hObject,handles)
end
guidata(hObject,handles)

% --- Executes during object creation, after setting all properties.
function dis1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to dis (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on button press in pushbutton8.
function pushbutton8_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
sliderValue=get(handles.slider_editText,'string');
d=str2double(sliderValue);
zl = str2double(get(handles.edit1,'string'));
z = (zl+1i*tan(2*pi*d))/(1+1i*zl*tan(2*pi*d));
y = 1/z;
aida = (z-1)/(z+1);
set(handles.zin1,'string',num2str(z));
set(handles.yin1,'string',num2str(y));
set(handles.aida1,'string',num2str(aida));
guidata(hObject,handles)

% --- Executes on selection change in dis2.
function dis2_Callback(hObject, eventdata, handles)%双支节部分d2的选择
% hObject    handle to dis2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: contents = get(hObject,'String') returns dis2 contents as cell array
%   contents{get(hObject,'Value')} returns selected item from dis2
val = get(hObject,'Value');
str = get(hObject, 'String');
switch str{val};
case '1/8 λ'
    handles.current_d2 = 0.125;
case '1/4 λ'
    handles.current_d2 = 0.25;
case '3/8 λ'
    handles.current_d2 = 0.375;
end
guidata(hObject,handles)

% --- Executes during object creation, after setting all properties.
function dis2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to dis2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

%function dis_Callback(hObject, eventdata, handles)
% hObject    handle to dis (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of dis as text
%        str2double(get(hObject,'String')) returns contents of dis as a double
%d = str2double(get(hObject,'String'));
%if isnan(d)
 %   errordlg('You must input a number.');
  %  set(handles.dis, 'string', '');
%elseif (d<0)
 %   errordlg('Distance cannot be negative.');
  %  set(handles.dis, 'string', '');
%end;

% --- Executes on button press in pushbutton10.
function pushbutton10_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global pic2;
set(gcf,'WindowButtonMotionFcn','');
hold off;
subplot(pic2);
background;
hold on;
zl = str2double(get(handles.edit1,'string'));
gamma_yl=-(zl-1)/(zl+1);
r=abs(gamma_yl);    %这里是画弧的
t1=angle(gamma_yl);
d1 = handles.current_d1;
d2 = handles.current_d2;
yin = (1+1i*zl*tan(2*pi*d1))/(zl+1i*tan(2*pi*d1));
flag = 0;   %标志是否盲区
switch d2;
case 0.125
    if real(yin) >= 2
        flag = 1;
        errordlg('Blind Area!!!');
    else
    circle(0,0.5,0.5,'r','--');
    end
case 0.25
    if real(yin) >= 1
        flag = 1;
        errordlg('Blind Area!!!');
    else
    circle(-0.5,0,0.5,'r','--');
    end
case 0.375
    if real(yin) >= 2
        flag = 1;
        errordlg('Blind Area!!!');
    else
    circle(0,-0.5,0.5,'r','--');
    end
end
if flag == 0
plot(real(gamma_yl),imag(gamma_yl),'kx', 'MarkerSize',10,'LineWidth',2);
hold on;
text(real(gamma_yl)+0.05,imag(gamma_yl),'zl');
gamma_y1=(yin-1)/(yin+1);
t2=angle(gamma_y1);
if t1<0
    t1 = t1+2*pi;
end
if t2<0
    t2 = t2+2*pi;
end
if t1>t2
    t = t1:-0.01:t2;
else
    t = t1:-0.01:-t2;
end
x=0+cos(t)*r;
y=0+sin(t)*r;
subplot(pic2);
plot(x,y,'r','LineWidth',2);
pause(0.2);
plot(x,y,'w','LineWidth',2);
pause(0.2);
plot(x,y,'r','LineWidth',2);
pause(0.2);
plot(x,y,'w','LineWidth',2);
pause(0.2);
plot(x,y,'r','LineWidth',2);     
hold on;
plot(real(gamma_y1),imag(gamma_y1),'r>');
hold on;
g1 = real(yin);
syms x y g1_;
switch d2;
case 0.125
    [x,y]=solve('(x-g1_/(1+g1_))^2+y^2 = 1/(1+g1_)^2' , 'x^2+(y-1/2)^2=1/4');theta=0.5*pi;
case 0.25
    [x,y]=solve('(x-g1_/(1+g1_))^2+y^2 = 1/(1+g1_)^2' , '(x+1/2)^2+y^2=1/4');theta=pi;
case 0.375
    [x,y]=solve('(x-g1_/(1+g1_))^2+y^2 = 1/(1+g1_)^2' , 'x^2+(y+1/2)^2=1/4');theta=1.5*pi;
end
x(1)=subs(x(1),g1_,g1);
x(2)=subs(x(2),g1_,g1);
y(1)=subs(y(1),g1_,g1);
y(2)=subs(y(2),g1_,g1);
x1=eval(x(1));
x2=eval(x(2));
y1=eval(y(1));
y2=eval(y(2)); 
aida(1) = x1+1i*y1;
aida(2) = x2+1i*y2;
yina(1) = (1+aida(1))/(1-aida(1));
yina(2) = (1+aida(2))/(1-aida(2));
r1=1/(1+real(yina(1)));
r2=1/(1+real(yina(2)));
t1=atan(y1/(x1-real(yina(1))/(real(yina(1))+1)));
t2=atan(y2/(x2-real(yina(2))/(real(yina(2))+1)));
t0=atan(imag(gamma_y1)/(real(gamma_y1)-real(yina(1))/(real(yina(1))+1)));   %以下是将幅角都转化为大于零小于2pi的角
if x1-real(yina(1))/(real(yina(1))+1) < 0   %第二，三象限
    t1 = t1+pi;
end
if x2-real(yina(2))/(real(yina(2))+1) < 0
    t2 = t2+pi;
end
if real(gamma_y1)-real(yina(1))/(real(yina(1))+1)
    t0 = t0+pi;
end
if t1 < 0       %第四象限
    t1 = t1+2*pi;
end
if t2 < 0
    t2 = t2+2*pi;
end
if t0 < 0
    t0 = t0+2*pi;
end      %一下决定弧的走向，使它沿较短的边画
if t1>t0
    if t1-t0<pi
        ta = t0:0.01:t1;
    else
        t1 = t1-2*pi;
        ta = t1:0.01:t0;
    end
else
    if t0-t1<pi
        ta = t1:0.01:t0;
    else
        t0 = t0-2*pi;
        ta = t0:0.01:t1;
        t0 = t0+2*pi;
    end
end
if t2>t0
    if t2-t0<pi
        tb = t0:0.01:t2;
    else
        t2 = t2-2*pi;
        tb = t2:0.01:t0;
    end
else
    if t0-t2<pi
        tb = t2:0.01:t0;
    else
        t0 = t0-2*pi;
        tb = t0:0.01:t2;
    end
end
xa=real(yina(1))/(1+real(yina(1)))+cos(ta)*r1;
ya=0+sin(ta)*r1;
xb=real(yina(2))/(1+real(yina(2)))+cos(tb)*r2;
yb=0+sin(tb)*r2;
subplot(pic2);
plot(xa,ya,'b','LineWidth',2);
hold on;
plot(xb,yb,'b','LineWidth',2);
pause(0.2); %延时显示
plot(xa,ya,'w','LineWidth',2);
plot(xb,yb,'w','LineWidth',2);
pause(0.2);
plot(xa,ya,'b','LineWidth',2);
hold on;
plot(xb,yb,'b','LineWidth',2);
pause(0.2);
plot(xa,ya,'w','LineWidth',2);
plot(xb,yb,'w','LineWidth',2);
pause(0.2);
plot(xa,ya,'b','LineWidth',2);
hold on;
plot(xb,yb,'b','LineWidth',2);
hold on;
plot(x1,y1,'b<');
hold on;
plot(x2,y2,'b<');
pause(0.2);

circle(0.5,0,0.5,'g','-');
tc=angle(aida(1));
td=angle(aida(2));
rc=abs(aida(1));
rd=abs(aida(2));
t3=tc:-0.01:(tc-theta);
t4=td:-0.01:(td-theta);
plot(rc*cos(t3),rc*sin(t3),'c','LineWidth',2);
plot(rd*cos(t4),rd*sin(t4),'c','LineWidth',2);
pause(0.2);
plot(rc*cos(t3),rc*sin(t3),'w','LineWidth',2);
plot(rd*cos(t4),rd*sin(t4),'w','LineWidth',2);
pause(0.2);
plot(rc*cos(t3),rc*sin(t3),'c','LineWidth',2);
plot(rd*cos(t4),rd*sin(t4),'c','LineWidth',2);
pause(0.2);
plot(rc*cos(t3),rc*sin(t3),'w','LineWidth',2);
plot(rd*cos(t4),rd*sin(t4),'w','LineWidth',2);
pause(0.2);
plot(rc*cos(t3),rc*sin(t3),'c','LineWidth',2);
plot(rd*cos(t4),rd*sin(t4),'c','LineWidth',2);
plot(rc*cos(tc-theta),rc*sin(tc-theta),'cp');
plot(rd*cos(td-theta),rd*sin(td-theta),'cp');
hold off;

b1(1) = yina(1)-yin;
L1(1) = distance(0,1/b1(1));
b1(2) = yina(2)-yin;
L1(2) = distance(0,1/b1(2));

yin3(1) = (yina(1)+1i*tan(2*pi*d2))/(1+1i*tan(2*pi*d2)*yina(1));
yin3(2) = (yina(2)+1i*tan(2*pi*d2))/(1+1i*tan(2*pi*d2)*yina(2));

b2(1) = -1i*imag(yin3(1));
b2(2) = -1i*imag(yin3(2));
L2(1) = distance(0,1/b2(1));
L2(2) = distance(0,1/b2(2));

set(handles.L1_1,'string',num2str(L1(1)))
set(handles.L1_2,'string',num2str(L1(2)))
set(handles.L2_1,'string',num2str(L2(1)))
set(handles.L2_2,'string',num2str(L2(2)))
%guidata(hObject,handles)
end
set(gcf,'WindowButtonMotionFcn',@draw);
guidata(hObject,handles)

% --------------------------------------------------------------------
function uipanel12_SelectionChangeFcn(hObject, eventdata, handles)
% hObject    handle to uipanel12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global input;
if hObject == handles.zz
    input = 'z';
else
    input = 'y';
end
guidata(hObject,handles)

% --- Executes on button press in pushbutton11.
function pushbutton11_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
zl = str2double(get(handles.edit1,'string'));
aida = (zl-1)/(zl+1);
rou = (1+abs(aida))/(1-abs(aida));
angle_a = angle(aida);
if angle_a<0
    angle_a = angle_a+2*pi;
end;
zmax = angle_a/4/pi;
if angle_a<pi
    zmin = (angle_a+pi)/4/pi;
else
    zmin = (angle_a-pi)/4/pi;
end;
set(handles.text5,'string',zmax)
set(handles.text6,'string',zmin)
set(handles.aida,'string',abs(aida))
set(handles.rou,'string',rou)
background;
hold on;
plot(real(aida),imag(aida),'x');
circle(0,0,abs(aida),'r','-');
circle(real(zl)/(1+real(zl)),0,1/(1+real(zl)),'g','-');
arch_x(imag(zl),'b');
hold off;
guidata(hObject,handles)


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over text51.
function text51_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to text51 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on mouse press over axes background.
function smith_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to smith (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes during object creation, after setting all properties.
function smith_CreateFcn(hObject, eventdata, handles)
% hObject    handle to smith (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate smith


% --- Executes during object creation, after setting all properties.
function wave_CreateFcn(hObject, eventdata, handles)
% hObject    handle to wave (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate wave


% --- Executes on slider movement.
function slider4_Callback(hObject, eventdata, handles)
% hObject    handle to slider4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
global  text5 text6;
slider4Value=get(handles.slider4,'Value');
set(handles.slider_editText,'string',num2str(slider4Value));
d=slider4Value;
zl = str2double(get(handles.edit1,'string'));
z = ((zl+1i*tan(2*pi*d))/(1+1i*zl*tan(2*pi*d)));
T=(zl-1)/(zl+1);
x = real(T);  %得到实部
y = imag(T);  %得到虚部，
if x^2+y^2 < 1   %判定选定的点是否正确
background;
hold on;
axis([-1,1,-1,1]);
aida = x+y*1i;   %得到选定的点所在的反射系数圆
angle_a = angle(aida);  %反射系数圆的相角
if angle_a<0
    angle_a = angle_a+2*pi;
end;
zmax = angle_a/4/pi;
if angle_a<pi
    zmin = (angle_a+pi)/4/pi;
else
    zmin = (angle_a-pi)/4/pi;
end;
set(text5,'string',zmax);
set(text6,'string',zmin);
r1 = abs(aida); %反射系数的模，反射系数圆的半径
r2 = 1/(1+real(z));%电阻圆的半径
z_real=real(z);
plot(x,y,'x');
circle(0,0,r1,'r','-');  %画选定点的反射系数圆
circle(z_real/(1+z_real),0,r2,'g','-');  %画选定点的电阻圆
Z_Image=imag(z);
if Z_Image == 0   %画选定点的电抗圆
    plot([-1 1],[0 0],'color',[0 0 1]);
else
    Draw_Arch(1/Z_Image,[0 0 1]);
end 
hold off;
end
guidata(hObject,handles)


% --- Executes during object creation, after setting all properties.
function slider4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
usewhitebg = 1;
if usewhitebg
    set(hObject,'BackgroundColor',[.9 .9 .9]);
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end


function slider_editText_Callback(hObject, eventdata, handles)
% hObject    handle to slider_editText (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of slider_editText as text
%        str2double(get(hObject,'String')) returns contents of slider_editText as a double
sliderValue=get(handles.slider_editText,'string');
sliderValue=str2double(sliderValue);

if(isempty(sliderValue)||sliderValue<0)
    set(handles.slider4,'Value',0);
    set(handles.slider_editText,'string','0');
else
    set(handles.slider4,'Value',sliderValue);
end
guidata(hObject,handles)


% --- Executes during object creation, after setting all properties.
function slider_editText_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_editText (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% Draw an arch
function Draw_Arch(y0,color)
x0=1;
if y0>0
    a=2*atan(-(y0+1)/(y0-1));
    if a>0
        t1=a;
    else
        t1=2*pi+a;
    end
    t2=1.5*pi;
else
    t1=0.5*pi;
    t2=pi-2*atan((y0+1)/(y0-1));
end
    t=t1:0.01:t2;
    x=x0+cos(t)*abs(y0);
    y=y0+sin(t)*abs(y0);
    plot(x,y,'-','color',color,'HitTest','off');


% --- Executes during object creation, after setting all properties.
% function pushbutton8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pushbutton8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns call



function edit14_Callback(hObject, eventdata, handles)
% hObject    handle to edit14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit14 as text
%        str2double(get(hObject,'String')) returns contents of edit14 as a double

%edit14=get(handles.edit14,'string');
%edit14Value=str2num(edit14);

%if(isempty(edit14Value)||edit14Value<0)
%    set(handles.edit14,'string','0');
%end
%guidata(hObject,handles)


% --- Executes during object creation, after setting all properties.
function edit14_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton13.
function pushbutton13_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global input;
edit14=get(handles.edit14,'string');
edit14Value=str2double(edit14);
edit20=get(handles.edit20,'string');
edit20Value=str2double(edit20);
Zc=edit14Value+1i*edit20Value;
edit16=get(handles.edit16,'string');
edit16Value=str2double(edit16);
edit21=get(handles.edit21,'string');
edit21Value=str2double(edit21);
Zl=edit16Value+1i*edit21Value;
zl=Zl/Zc;
aida = (zl-1)/(zl+1);
rou = (1+abs(aida))/(1-abs(aida));
angle_a = angle(aida);
if angle_a<0
    angle_a = angle_a+2*pi;
end;
zmax = angle_a/4/pi;
if angle_a<pi
    zmin = (angle_a+pi)/4/pi;
else
    zmin = (angle_a-pi)/4/pi;
end;

if input == 'z'
    if isnan(zl)
        errordlg('You must input a number.');
        set(handles.edit1, 'string', '');
    elseif real(zl<0)
        errordlg('The real part cannot be negative.');
        set(handles.edit1, 'string', '');
    else
        set(handles.edit1, 'string', num2str(zl));
    end
else
    if isnan(zl)
        errordlg('You must input a number.');
        set(handles.edit1, 'string', '');
    elseif real(zl<0)
        errordlg('The real part cannot be negative.');
        set(handles.edit1, 'string', '');
    else
        set(handles.edit1, 'string', num2str(1/zl));
    end   
end

set(handles.text5,'string',zmax)
set(handles.text6,'string',zmin)
set(handles.aida,'string',abs(aida))
set(handles.rou,'string',rou)
background;
hold on;
plot(real(aida),imag(aida),'x');
circle(0,0,abs(aida),'r','-');
circle(real(zl)/(1+real(zl)),0,1/(1+real(zl)),'g','-');
arch_x(imag(zl),'b');
hold off;
guidata(hObject,handles)




function edit16_Callback(hObject, eventdata, handles)
% hObject    handle to edit16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit16 as text
%        str2double(get(hObject,'String')) returns contents of edit16 as a double


% --- Executes during object creation, after setting all properties.
function edit16_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end





function edit20_Callback(hObject, eventdata, handles)
% hObject    handle to edit20 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit20 as text
%        str2double(get(hObject,'String')) returns contents of edit20 as a double


% --- Executes during object creation, after setting all properties.
function edit20_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit20 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit21_Callback(hObject, eventdata, handles)
% hObject    handle to edit21 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit21 as text
%        str2double(get(hObject,'String')) returns contents of edit21 as a double


% --- Executes during object creation, after setting all properties.
function edit21_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit21 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton18.
function pushbutton18_Callback(hObject, eventdata, handles)
global flag0;
flag0=0;
bodao;
close smith;

% --- Executes on button press in pushbutton19.
function pushbutton19_Callback(hObject, eventdata, handles)
global flag0;
flag0=0;
antenna;
close smith;

% --- Executes on button press in pushbutton20.
function pushbutton20_Callback(hObject, eventdata, handles)
global flag0;
flag0=0;
main;
close smith;


% --- Executes on button press in pushbutton21.
function pushbutton21_Callback(hObject, eventdata, handles)
set(handles.text_11,'Visible','on');
pause(1);
close smith;
