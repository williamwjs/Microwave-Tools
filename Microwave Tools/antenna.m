function varargout = antenna(varargin)
% ANTENNA MATLAB code for antenna.fig
%      ANTENNA, by itself, creates a new ANTENNA or raises the existing
%      singleton*.
%
%      H = ANTENNA returns the handle to a new ANTENNA or the handle to
%      the existing singleton*.
%
%      ANTENNA('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in ANTENNA.M with the given input arguments.
%
%      ANTENNA('Property','Value',...) creates a new ANTENNA or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before antenna_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to antenna_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help antenna

% Last Modified by GUIDE v2.5 10-Jul-2013 14:42:27

% Begin initialization code - DO NOT EDIT



gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @antenna_OpeningFcn, ...
                   'gui_OutputFcn',  @antenna_OutputFcn, ...
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

% --- Executes just before antenna is made visible.
function antenna_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to antenna (see VARARGIN)

% Choose default command line output for antenna
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes antenna wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = antenna_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;
ssize=get(0,'screensize');
set(hObject,'position',ssize);





function zhenzi_Callback(hObject, eventdata, handles)
% hObject    handle to zhenzi (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of zhenzi as text
%        str2double(get(hObject,'String')) returns contents of zhenzi as a double


% --- Executes during object creation, after setting all properties.
function zhenzi_CreateFcn(hObject, eventdata, handles)
% hObject    handle to zhenzi (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in antenna_begin1.


% --- Executes on button press in antenna_aut.
function antenna_aut_Callback(hObject, eventdata, handles)
% hObject    handle to antenna_aut (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of antenna_aut
%if(get(handles.antenna_begin1, 'Value') == 1)
axes(handles.antenna_axes1);
cla;
h=0;
global judge;
judge=1;
if get(handles.antenna_aut, 'Value') == 1
while(1)
   if(judge==0)
   break;
   end
   h=h+0.1;
   H=num2str(h);
   set(handles.zhenzi,'string',H);
   x=0:0.01:2*pi;
   F=(cos(2*pi*h*cos(x))-cos(2*pi*h))./sin(x);
   F=abs(F);
   f=F/max(F);  %归一化方向图
   polar(x,f,'r');
   if(h>6) h=0;
   end
   pause(0.25);
end
else judge=0;
end


function antenna_zhenyuan_Callback(hObject, eventdata, handles)
% hObject    handle to antenna_zhenyuan (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of antenna_zhenyuan as text
%        str2double(get(hObject,'String')) returns contents of antenna_zhenyuan as a double



function antenna_distance_Callback(hObject, eventdata, handles)
% hObject    handle to antenna_distance (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of antenna_distance as text
%        str2double(get(hObject,'String')) returns contents of antenna_distance as a double


% --- Executes during object creation, after setting all properties.
function antenna_distance_CreateFcn(hObject, eventdata, handles)
% hObject    handle to antenna_distance (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in antenna_begin2.
function antenna_begin2_Callback(hObject, eventdata, handles)
% hObject    handle to antenna_begin2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of antenna_begin2
%antenna_d  阵元间距
%number  阵元数
if(get(handles.antenna_begin2, 'Value') == 1)
 axes(handles.antenna_axes1);
 set(handles.antenna_aut,'value',0);
 global judge;
 judge=0;
fai=str2double(get(handles.antenna_fai,'String'));
fai=fai/180*pi;
d = str2double(get(handles.antenna_distance, 'String'));
number = str2num(get(handles.antenna_zhenyuan, 'String'));
a=0:1:number-1;
if(get(handles.antenna_x, 'Value') == 1)
            m=0;        
            for x=0:0.05:2*pi
            b=a.*(2*pi*d*1i);
            c=b.*(sin(x)*cos(fai));
            FA=sum(exp(c));
            m=m+1;
            F(m)=abs(sqrt(1-(sin(x)*cos(fai))^2)*FA);   %归一化方向图
            Fa(m)=abs(FA);          %归一化阵方向图
            end
else if(get(handles.antenna_y, 'Value') == 1)
             m=0;        
            for x=0:0.05:2*pi
            b=a.*(2*pi*d*1i);
            c=b.*(sin(x)*sin(fai));
            FA=sum(exp(c));
            m=m+1;
            F(m)=abs(sqrt(1-(sin(x)*sin(fai))^2)*FA);   %归一化方向图
            Fa(m)=abs(FA);          %归一化阵方向图
            end
    else if(get(handles.antenna_z, 'Value') == 1)
            m=0;        
            for x=0:0.05:2*pi
            b=a.*(2*pi*d*1i);
            c=b.*cos(x);
            FA=sum(exp(c));
            m=m+1;
            F(m)=abs(sin(x)*FA);   %归一化方向图
            Fa(m)=abs(FA);          %归一化阵方向图
            end
        end
      end
end
            X=0:0.05:2*pi;
            f=F./max(F);
            fa=Fa./max(Fa);
            if (get(handles.antenna_zhentu,'Value')==1)
            polar(X,fa,'r');
            end ;
            if (get(handles.antenna_fangtu,'Value')==1)
            polar(X,f,'r');
            end ;   
end


%点击计算显示方向图
% --- Executes on button press in antenna_begin1.
function antenna_begin1_Callback(hObject, eventdata, handles)
% hObject    handle to antenna_begin1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of antenna_begin1
axes(handles.antenna_axes1);
cla;
 global judge;
 judge=0;
set(handles.antenna_aut,'value',0);
h=str2double(get(handles.zhenzi, 'String'));
x=0:0.01:2*pi;
F=(cos(2*pi*h*cos(x))-cos(2*pi*h))./sin(x);
F=abs(F);
f=F/max(F);  %归一化方向图
polar(x,f,'r');


function antenna_fai_Callback(hObject, eventdata, handles)
% hObject    handle to antenna_fai (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of antenna_fai as text
%        str2double(get(hObject,'String')) returns contents of antenna_fai as a double


% --- Executes during object creation, after setting all properties.
function antenna_fai_CreateFcn(hObject, eventdata, handles)
% hObject    handle to antenna_fai (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton7.
function pushbutton7_Callback(hObject, eventdata, handles)
global judge;
judge=0;
bodao;
close antenna;


% --- Executes on button press in pushbutton8.
function pushbutton8_Callback(hObject, eventdata, handles)
global judge;
judge=0;
smith;
close antenna;


% --- Executes on button press in pushbutton9.
function pushbutton9_Callback(hObject, eventdata, handles)
global judge;
judge=0;
main;
close antenna;


% --- Executes on button press in pushbutton10.
function pushbutton10_Callback(hObject, eventdata, handles)
global judge;
judge=0;
set(handles.text_11,'Visible','on');
pause(1);
close antenna;
