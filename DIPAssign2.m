function varargout = DIPAssign2(varargin)
% DIPASSIGN2 MATLAB code for DIPAssign2.fig
%      DIPASSIGN2, by itself, creates a new DIPASSIGN2 or raises the existing
%      singleton*.
%
%      H = DIPASSIGN2 returns the handle to a new DIPASSIGN2 or the handle to
%      the existing singleton*.
%
%      DIPASSIGN2('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in DIPASSIGN2.M with the given input arguments.
%
%      DIPASSIGN2('Property','Value',...) creates a new DIPASSIGN2 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before DIPAssign2_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to DIPAssign2_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help DIPAssign2

% Last Modified by GUIDE v2.5 11-Dec-2021 10:50:09

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @DIPAssign2_OpeningFcn, ...
                   'gui_OutputFcn',  @DIPAssign2_OutputFcn, ...
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

%====================================
%global variable for histogram
function setGlobalx(val)
global x
x = val;

function r = getGlobalx
global x
r = x;
%====================================
function setGlobalFile(val)
global p
p = val;
function z = getGlobalFile
global p
z = p;
%====================================

% --- Executes just before DIPAssign2 is made visible.
function DIPAssign2_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to DIPAssign2 (see VARARGIN)

% Choose default command line output for DIPAssign2
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes DIPAssign2 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = DIPAssign2_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu1


% --- Executes during object creation, after setting all properties.
function popupmenu1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton22.
function pushbutton22_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton22 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton23.
function pushbutton23_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton23 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton24.
function pushbutton24_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton24 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton25.
function pushbutton25_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton25 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton26.
function pushbutton26_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton26 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton27.
function pushbutton27_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton27 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in Upload.
function Upload_Callback(hObject, eventdata, handles)
% hObject    handle to Upload (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%[file,path] = uigetfile('*.*');
a=uigetfile('*.*');
setGlobalFile(a);
a=imread(a);
axes(handles.axes1);
imshow(a);
setappdata(0,'a',a);


% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
a=getappdata(0,'a');
agray=rgb2gray(a);
axes(handles.axes2);
imshow(agray);


% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton6.
function pushbutton6_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
a=getappdata(0,'a');
a=rgb2gray(a);
axes(handles.axes1);
imhist(a);
%imageinfo(a);


% --- Executes on button press in pushbutton7.
function pushbutton7_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton8.
function pushbutton8_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton9.
function pushbutton9_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton18.
function pushbutton18_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton18 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton19.
function pushbutton19_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton19 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton28.
function pushbutton28_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton28 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton29.
function pushbutton29_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton29 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton30.
function pushbutton30_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton30 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton31.
function pushbutton31_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton31 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton32.
function pushbutton32_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton32 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton33.
function pushbutton33_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton33 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton34.
function pushbutton34_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton34 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton35.
function pushbutton35_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton35 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton36.
function pushbutton36_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton36 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton37.
function pushbutton37_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton37 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton38.
function pushbutton38_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton38 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton39.
function pushbutton39_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton39 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes on button press in pushbutton40.
function pushbutton40_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton40 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton41.
function pushbutton41_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton41 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton42.
function pushbutton42_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton42 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton43.
function pushbutton43_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton43 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
I = get(handles.popupmenu2,'value');
switch I
    case 2
        a=getappdata(0,'a');
        biLinearInter=imresize(a,0.3,'bilinear');
        axes(handles.axes2);
        imshow(biLinearInter);
        setGlobalx(biLinearInter);
    case 3
        a=getappdata(0,'a');
        nearInter=imresize(a,0.3,'nearest');
        axes(handles.axes2);
        imshow(nearInter);
        setGlobalx(nearInter);
    case 4
        a=getappdata(0,'a');
        bcInter=imresize(a,0.3,'bicubic');
        axes(handles.axes2);
        imshow(bcInter);
        setGlobalx(bcInter);
    otherwise
        disp('no filter');
end


% --- Executes on selection change in popupmenu2.
function popupmenu2_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu2 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu2


% --- Executes during object creation, after setting all properties.
function popupmenu2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton56.
function pushbutton56_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton56 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
r = getGlobalx;
b=rgb2gray(r);
axes(handles.axes2);
imhist(b);


% --- Executes on button press in pushbutton52.
function pushbutton52_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton52 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton53.
function pushbutton53_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton53 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton54.
function pushbutton54_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton54 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton55.
function pushbutton55_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton55 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton58.
function pushbutton58_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton58 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
I = get(handles.popupmenu4,'value');
switch I
    case 2
        a=getappdata(0,'a');
        salt=imnoise(a,'salt & pepper',0.02);
        axes(handles.axes2);
        imshow(salt);
        setGlobalx(salt);
    case 3
        a=getappdata(0,'a');
        gau=imnoise(a,'gaussian',0,0.02); %Gassian Noise with mean 0 and vaiance 0.02
        axes(handles.axes2);
        imshow(gau);
        setGlobalx(gau);
    case 4
        a=getappdata(0,'a');
        a=rgb2gray(a);
        a=imresize(a,[256,256],'nearest');
        mynoise=6+(sqrt(255)*randn(256,256));
        mynoiseimg=double(a) + mynoise;
        axes(handles.axes2);
        imshow(mynoiseimg,[]);
        setGlobalx(mynoiseimg);
    case 5
        a=getappdata(0,'a');
        a=rgb2gray(a);
        a=imresize(a,[256,256],'nearest');
        [x,y]=meshgrid(1:256,1:256);
        mysinesoidalnoise=15*sin(2*pi/14*x+2*pi/14*y);
        mynoiseimg1=double(a) + mysinesoidalnoise;
        axes(handles.axes2);
        imshow(mynoiseimg1,[]);
        setGlobalx(mynoiseimg1);
    otherwise
        disp('no noise');
end


% --- Executes on selection change in popupmenu4.
function popupmenu4_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu4 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu4


% --- Executes during object creation, after setting all properties.
function popupmenu4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton59.
function pushbutton59_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton59 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
I = get(handles.popupmenu5,'value');
switch I
    case 2
        a=getappdata(0,'a');
        img1=a;
        GaussianFilter = imgaussfilt(img1,2); %with std dev=2
        img2 = img1-GaussianFilter; 
        img3 = img1+img2;
        axes(handles.axes2);
        imshow(img3);
        setGlobalx(img3);
    case 3
        a=getappdata(0,'a');
        img1=a;
        GaussianFilter = imgaussfilt(img1,2); %with std dev=2
        img2 = img1-GaussianFilter; 
        img3 = img1+img2*4.5;
        axes(handles.axes2);
        imshow(img3);
        setGlobalx(img3);
    case 4
        a=getappdata(0,'a');
        f=a;
        w4 = fspecial('laplacian',0);
        f=im2double(f);
        f4=imfilter(f,w4,'replicate');
        img4=f-f4;
        axes(handles.axes2);
        imshow(img4);
        setGlobalx(img4);
    case 5
        a=getappdata(0,'a');
        f=a;
        w8 = [1,1,1;1,-8,1;1,1,1];
        f=im2double(f);
        f8=imfilter(f,w8,'replicate');
        img8=f-f8;
        axes(handles.axes2);
        imshow(img8);
        setGlobalx(img8);
    case 6
        a=getappdata(0,'a');
        a=rgb2gray(a);
        BoxFilter = conv2(single(a), ones(3)/9, 'same');
        axes(handles.axes2);
        imshow(BoxFilter, []);
        setGlobalx(BoxFilter);
    case 7
        a=getappdata(0,'a');
        a=rgb2gray(a);
        WtFilter = conv2(single(a), [1,2,1;2,4,2;1,2,1]/16, 'same');
        axes(handles.axes2);
        imshow(WtFilter, []);
        setGlobalx(WtFilter);
    otherwise
        disp('no noise');
end


% --- Executes on selection change in popupmenu5.
function popupmenu5_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu5 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu5


% --- Executes during object creation, after setting all properties.
function popupmenu5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton57.
function pushbutton57_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton57 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
I = get(handles.popupmenu3,'value');
switch I
    case 2
        %FFT2 CODE
        a=getappdata(0,'a');
        f=rgb2gray(a);
        % Apply filtering
        h = fspecial('sobel');
        PQ = paddedsize(size(f));
        F = fft2(double(f), PQ(1), PQ(2));
        H = fft2(double(h), PQ(1), PQ(2));
        F_fH = H.*F;
        ffi = ifft2(F_fH); 
        ffi = ffi(2:size(f,1)+1,2:size(f,2)+1);
        axes(handles.axes2);
        imshow(ffi,[]);
        setGlobalx(ffi);
    case 3
        %low ideal pass
        a=getappdata(0,'a');
        input_image = rgb2gray(a);
        [M,N] = size(input_image);
        FT_img = fft2(double(input_image));
        D0 = 30;
        u = 0:(M-1);
        idx = find(u>M/2);
        u(idx) = u(idx)-M;
        v = 0:(N-1);
        idy = find(v>N/2);
        v(idy) = v(idy)-N;
        [V, U] = meshgrid(v, u);
        D = sqrt(U.^2+V.^2);
        H = double(D<=D0);
        G = H.*FT_img;
        output_img = real(ifft2(double(G)));
        axes(handles.axes2);
        imshow(output_img, [ ]);
        setGlobalx(output_img);
    case 4
        %gaussian lps
        a=getappdata(0,'a');
        org_img = rgb2gray(a);
        football = imnoise(org_img,'salt & pepper',0.02); %'salt & pepper')
        PQ = paddedsize(size(football));
        D0 = 0.05*PQ(1);
        H = lpfilter('gaussian',PQ(1),PQ(2),D0);
        F = fft2(double(football),size(H,1),size(H,2));
        LPFS_football = H.*F;
        LPF_football = real(ifft2(LPFS_football));
        LPF_football = LPF_football(1:size(football,1),1:size(football,2));
        axes(handles.axes2);
        imshow(LPF_football,[])
        setGlobalx(LPF_football);
    case 5
        %ideal hpf
        a=getappdata(0,'a');
        input_image=rgb2gray(a);
        [M, N] = size(input_image);
        FT_img = fft2(double(input_image));
        D0 = 10;
        u = 0:(M-1);
        idx = find(u>M/2);
        u(idx) = u(idx)-M;
        v = 0:(N-1);
        idy = find(v>N/2);
        v(idy) = v(idy)-N;
        [V, U] = meshgrid(v, u);
        D = sqrt(U.^2+V.^2);
        H = double(D > D0);
        G = H.*FT_img;
        output_image = real(ifft2(double(G)));
        axes(handles.axes2);
        imshow(output_image, [ ]);
        setGlobalx(output_image);
    case 6
        %Butterworth LPF
        a=getappdata(0,'a');
        input_image=rgb2gray(a);
        [M, N] = size(input_image);
        FT_img = fft2(double(input_image));
        n = 2; 
        D0 = 20;
        u = 0:(M-1);
        v = 0:(N-1);
        idx = find(u > M/2);
        u(idx) = u(idx) - M;
        idy = find(v > N/2);
        v(idy) = v(idy) - N;
        [V, U] = meshgrid(v, u);
        D = sqrt(U.^2 + V.^2);
        H = 1./(1 + (D./D0).^(2*n));
        G = H.*FT_img;
        output_image = real(ifft2(double(G)));
        axes(handles.axes2);
        imshow(output_image, [ ]);
        setGlobalx(output_image);
    case 7
        %Butterworth HPF
        a=getappdata(0,'a');
        input_image=rgb2gray(a);
        [M, N] = size(input_image);
        FT_img = fft2(double(input_image));
        n = 2;
        D0 = 20;
        u = 0:(M-1);
        v = 0:(N-1);
        idx = find(u > M/2);
        u(idx) = u(idx) - M;
        idy = find(v > N/2);
        v(idy) = v(idy) - N;
        [V, U] = meshgrid(v, u);
        D = sqrt(U.^2 + V.^2);
        H = 1./(1 + (D./D0).^(2*n));
        G = H.*(1-FT_img);
        output_image = real(ifft2(double(G)));
        axes(handles.axes2);
        imshow(output_image, [ ]);
        setGlobalx(output_image);
    case 8
        %gaussian hps
        a=getappdata(0,'a');
        footBall=rgb2gray(a);
        PQ = paddedsize(size(footBall));
        D0 = 0.05*PQ(1);
        H = hpfilter('gaussian', PQ(1), PQ(2), D0);
        F=fft2(double(footBall),size(H,1),size(H,2));
        HPFS_football = H.*F;
        HPF_football=real(ifft2(HPFS_football));
        HPF_football=HPF_football(1:size(footBall,1), 1:size(footBall,2));
        axes(handles.axes2);
        imshow(HPF_football, []);
        setGlobalx(HPF_football);
    otherwise
        disp('no noise');
end


% --- Executes on selection change in popupmenu3.
function popupmenu3_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu3 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu3


% --- Executes during object creation, after setting all properties.
function popupmenu3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton61.
function pushbutton61_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton61 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
a=getappdata(0,'a');
axes(handles.axes1);
imshow(a);


% --- Executes on button press in pushbutton62.
function pushbutton62_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton62 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton63.
function pushbutton63_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton63 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
z = getGlobalFile;
info = imageinfo(z);
imageinfo(z,info)


% --- Executes on button press in pushbutton64.
function pushbutton64_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton64 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
r = getGlobalx;
info1 = imageinfo(r);
imageinfo(r,info1)
