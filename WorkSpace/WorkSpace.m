function varargout = WorkSpace(varargin)
% WORKSPACE MATLAB code for WorkSpace.fig
%      WORKSPACE, by itself, creates a new WORKSPACE or raises the existing
%      singleton*.
%
%      H = WORKSPACE returns the handle to a new WORKSPACE or the handle to
%      the existing singleton*.
%
%      WORKSPACE('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in WORKSPACE.M with the given input arguments.
%
%      WORKSPACE('Property','Value',...) creates a new WORKSPACE or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before WorkSpace_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to WorkSpace_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help WorkSpace

% Last Modified by GUIDE v2.5 30-Dec-2017 02:22:24

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @WorkSpace_OpeningFcn, ...
                   'gui_OutputFcn',  @WorkSpace_OutputFcn, ...
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


% --- Executes just before WorkSpace is made visible.
function WorkSpace_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to WorkSpace (see VARARGIN)
% Choose default command line output for WorkSpace
handles.output = hObject;

s = 'realTimePlotOff';
handles.s=s;
%handles.showData=0;
% Update handles structure
guidata(hObject, handles);

% UIWAIT makes WorkSpace wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = WorkSpace_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function l1Min_Callback(hObject, eventdata, handles)
% hObject    handle to l1Min (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of l1Min as text
%        str2double(get(hObject,'String')) returns contents of l1Min as a double


% --- Executes during object creation, after setting all properties.
function l1Min_CreateFcn(hObject, eventdata, handles)
% hObject    handle to l1Min (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function l1Max_Callback(hObject, eventdata, handles)
% hObject    handle to l1Max (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of l1Max as text
%        str2double(get(hObject,'String')) returns contents of l1Max as a double


% --- Executes during object creation, after setting all properties.
function l1Max_CreateFcn(hObject, eventdata, handles)
% hObject    handle to l1Max (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function angel1Min_Callback(hObject, eventdata, handles)
% hObject    handle to angel1Min (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of angel1Min as text
%        str2double(get(hObject,'String')) returns contents of angel1Min as a double


% --- Executes during object creation, after setting all properties.
function angel1Min_CreateFcn(hObject, eventdata, handles)
% hObject    handle to angel1Min (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function angel1Max_Callback(hObject, eventdata, handles)
% hObject    handle to angel1Max (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of angel1Max as text
%        str2double(get(hObject,'String')) returns contents of angel1Max as a double


% --- Executes during object creation, after setting all properties.
function angel1Max_CreateFcn(hObject, eventdata, handles)
% hObject    handle to angel1Max (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
s=handles.s;                                                        %get the global variable handles.s (s stores whether the program will plot in real time or normal)
showData = get(handles.DataInBase,'Value');                         %see if the user wants to send the data (X,Y,Z) to the base workspace
set(handles.plotting,'Visible','on');                               %show a "Plotting" sign in the GUI to tell the user that the program is working when it takes too long to show results
drawnow                                                             %show any changes to the GUI now (Matlab usually shows the changes at the end of the function)
%pause(0.1);                                                        %this line would have achived the same result but will take 0.1 second
if get(handles.pop,'Value') == 1 && strcmp(s ,'realTimePlotOff')    %if the robot is "2D RP" and no real time plotting
    cla(handles.axes1)                                              %clear any previous drawing on the axes
    %The following lines are used to Read the values from the GUI Edit Boxes
    Drawing_Step=pi*str2double(get(handles.draw_step,'string'))/180;%Assign the fixed drawing step that the user chose from the GUI
    Len_Step    =str2double(get(handles.len_step,'string'));        %Assign the fixed length step that the user chose from the GUI
    l1min=str2double(get(handles.l1Min,'string'));                  %Assign the minimum length of the prismatic joint
    l1max=str2double(get(handles.l1Max,'string'));                  %Assign the maximun length of the prismatic joint
    theta1min=pi*str2double(get(handles.angel1Min,'string'))/180;   %Assign the minimum angle of the revolute joint (convert the angle from degree to radian)
    theta1max=pi*str2double(get(handles.angel1Max,'string'))/180;   %Assign the maximum angle of the revolute joint (the GUI angles are in degree and Matlab is in radian)
    
    % calculate The Value of Each point
    view(handles.axes1,2);                                          %tell axes1 to view in 2D (it's an unnecessary line to be honest, Matlab converts it automatically)
    %hold on;                                                       %tell axes1 to not erase previous points and append new points to the existing ones(no need to hold on since it's not real time and all the points are plotted at the same time)
    size_x = ((l1max - l1min)/Len_Step) + 1;                        %calculate how many different possible points will the prismatic joint has (+1 is becase matlabe starts from 1 not 0)
%     size_x = l1max - l1min
%     size_x = size_x / Len_Step
%     size_x = size_x + 1
    size_y = ((theta1max - theta1min)/Drawing_Step) + 1;            %calculate how many different possible points will the revolute joint has (+1 is becase matlabe starts from 1 not 0)
%     size_y = (theta1max - theta1min)
%     size_y = size_y / Drawing_Step
%     size_y = size_y +1
    size = ceil(size_x) * ceil(size_y);                             %calculate the total size of all the possible points of the revolute and the prismatic joints when combined
    x = zeros([1,size]);                                            %allocate a memory array for the points (for the x axis points)
    y = zeros([1,size]);                                            %allocate a memory array for the points (for the y axis points)
    count = 0;                                                      %allocate variable count to hold the current index of the x,y point
	x(1)=0;                                                         %kinda useless, i was just checking the syntax
	y(1)=0;
    for len =l1min:Len_Step:l1max                                   %loop for all possible lengths of the prismatic rod
        for ang =theta1min:Drawing_Step:theta1max                   %loop for all possible angles of the revolute joint
			%x=[x len*cos(ang)];                                    %this is the old method i used when i didn't allocate space for the points in advance but matlab kept annoying me saying it's slow so i allocated the space :\
            %y=[y len*sin(ang)];
            count = count+1;
            x(count)=len*cos(ang);                                  %very simple, just look at the drawing of the robot and you'll understand
            y(count)=len*sin(ang);
        end
    end
    plot(handles.axes1,x,y,'*')                                     %now plot all the x,y points
    drawnow;
    %sending data to base workspace, Note that Plotting message that appears in the gui will remain visible while data is being copied to base's workspace
    if showData == 1
        assignin('base','size_x',size_x);                           %assignin(workspace name, 'what do you want to call the variable in the new workspace', name of the variable in this workspace)
        assignin('base','size_y',size_y);
        assignin('base','size',size);
        assignin('base','count',count);
        assignin('base','Len_Step',Len_Step);
        assignin('base','Drawing_Step',Drawing_Step);
        assignin('base','l1max',l1max);
        assignin('base','l1min',l1min);
        assignin('base','theta1max',theta1max);
        assignin('base','theta1min',theta1min);
        assignin('base','x',x);
        assignin('base','y',y);
    end
    
elseif get(handles.pop,'Value') == 1 && strcmp(s ,'realTimePlotOn')     %if the robot is "2D RP" with real time plotting
    cla(handles.axes1)                                                  % clear any previous drawing on the axes
    Drawing_Step=pi*str2double(get(handles.draw_step,'string'))/180;    % Assign a fixed drawing step
    Len_Step    =str2double(get(handles.len_step,'string'));            %L1 len step
    %The following lines are used to Read the values in GUI Edit Boxes
    l1min=str2double(get(handles.l1Min,'string'));
    l1max=str2double(get(handles.l1Max,'string'));
    theta1min=pi*str2double(get(handles.angel1Min,'string'))/180;
    theta1max=pi*str2double(get(handles.angel1Max,'string'))/180;
    
    % calculate The Value of Each point
    view(handles.axes1,2);
    hold on                                                             %tell axes1 to not erase previous points and append new points to the existing ones
    for len =l1min:Len_Step:l1max
        for ang =theta1min:Drawing_Step:theta1max
            x=len*cos(ang);
            y=len*sin(ang);
            plot(handles.axes1,x,y,'*')
            %drawnow;                                                   %this will update the axis for each point at a time and will be very very slow
        end
        drawnow;                                                        %this is the change from the NO RealTime, I'm forcing matlabe to plot or update the axis (this method slow)
    end
    
elseif get(handles.pop,'Value') == 2 && strcmp(s,'realTimePlotOff')     %if the robot is "3D RRP" and no real time plotting
    cla(handles.axes1) % clear any previous drawing on the axes
    Drawing_Step=str2double(get(handles.draw_step,'string'));           %Assign the fixed drawing step(for both revolute joints) that the user chose from the GUI
    Len_Step    =str2double(get(handles.len_step,'string'));            %Assign the fixed length step that the user chose from the GUI
    l1min=str2double(get(handles.l1Min,'string'));                      %Assign the minimum length of the prismatic joint
    l1max=str2double(get(handles.l1Max,'string'));                      %Assign the maximum length of the prismatic joint
    theta1min=pi*str2double(get(handles.angel1Min,'string'))/180;       %Assign the minimum angle of the first revolute joint (and convert the angle from degree to radian)
    theta1max=pi*str2double(get(handles.angel1Max,'string'))/180;       %Assign the maximum angle of the first revolute joint (and convert the angle from degree to radian)
    theta2min=pi*str2double(get(handles.angel2Min,'string'))/180;       %Assign the minimum angle of the second revolute joint (and convert the angle from degree to radian)
    theta2max=pi*str2double(get(handles.angel2Max,'string'))/180;       %Assign the maximum angle of the second revolute joint (and convert the angle from degree to radian)
   
    view(handles.axes1,3);                                              %tell axes1 to view in 3D
    %hold on                                                            %no need to hold on since it's not real time and all the points are plotted at the same time
    
    size_x = ((l1max - l1min)/Len_Step) +1;                             %same as before calculate how many possible points for each individual joint
    size_y = ((theta1max - theta1min)/Drawing_Step);
    size_z = ((theta2max - theta2min)/Drawing_Step);
    size = ceil(size_x)*ceil(size_y)*ceil(size_z);                      %i rounded the sizes becaue the angles were converted to radian (has flotting point) and the size of the array must be an integer
    
    %allocate the array for x,y and z
    x = zeros([1,size]);
    y = zeros([1,size]);
    z = zeros([1,size]);
    
%     x = zeros([1,1000000]);                                           %you could have allocated a large array for the points if you didn't want to calculate the exact number [all the extra points will be at (0,0,0)]
%     y = zeros([1,1000000]);
%     z = zeros([1,1000000]);
    count = 0;
	x(1)=0;
	y(1)=0;
    z(1)=0;
    for len = l1min:Len_Step:l1max
        for ang1 =theta1min:Drawing_Step:theta1max
            for ang2 =theta2min:Drawing_Step:theta2max
                count = count +1;
                z(count) = len * cos(ang1);               %ang1 with z
                x(count) = len * sin(ang1) * cos(ang2);   %ang2 with x
                y(count) = len * sin(ang1) * sin(ang2);
                %scatter3(handles.axes1,x,y,z,'*');                     %i don't remember what scatter3 does exactly but i think it gave random color for the points, you can uncomment it and see
            end
        end
    end
    cla(handles.axes1);
    plot3(handles.axes1,x,y,z,'*');                                     %i used plot3 cuz it's a 3d plot, plot will give you an error
    %if you checked the "grid on" box
    if get(handles.grid,'value') == 1 
        hold on                                                         %don't erase what i've plotted before (yes i'm planning to take over the world)
        plot3(handles.axes1,x,y,z*0,'Color', [0.6196 0.6196 0.6196]);   %plot the same points but with an altitude of zero (z=0) and set the color to gray (i'm trying to cast a shadow for the points and yes i stole the idea from the robotics toolbox)
        grid on
        xlim=get(handles.axes1,'xlim');                                 %get the scope of the X-axis e.g. X axis starts from 0 and ends at 10
        ylim=get(handles.axes1,'ylim');                                 %get the scope of the Y-axis e.g. Y axis starts from -5 and ends at 5
        xline = linspace(floor(xlim(1)),ceil(xlim(2)),10);              %make 10 points with equal distances between the start of the X axis to the end (round xlim to make sure they're integers)
        yline = linspace(floor(ylim(1)),ceil(ylim(2)),10);              %same thing with Y axis
        [X,Y] = meshgrid(xline,yline);                                  %i honstly don't know how to explain this is words so you should see the videos for this one
        Z = X * 0;                                                      %set the altitude to zero (i want to plot the plane at z=0) [i used X*0 because i need z array to be the same number as X and Y otherwise i'll have an error)
        surf(X,Y,Z);                                                    %now use the points we generated to plot a surface (the green plane at z=0 and yes i stole the idea from the toolbox again, you can change the color by the way just do it like you would in plot or see it's help)
        xlabel(handles.axes1,'X');                                      %label the x axis with 'X'
        ylabel(handles.axes1,'Y');
        zlabel(handles.axes1,'Z');
        drawnow                                                         %plot all of what we did before now (also useless, i honestly don't know why i add those useless lines)
        hold off
    end
    %sending data to base workspace, Note that Plotting message that appears in the gui will remain visible while data is being copied to base's workspace
    if showData == 1
        assignin('base','size_x',size_x);
        assignin('base','size_y',size_y);
        assignin('base','size_z',size_z);
        assignin('base','size',size);
        assignin('base','count',count);
        assignin('base','Len_Step',Len_Step);
        assignin('base','Drawing_Step',Drawing_Step);
        assignin('base','l1max',l1max);
        assignin('base','l1min',l1min);
        assignin('base','theta1max',theta1max);
        assignin('base','theta1min',theta1min);
        assignin('base','theta2max',theta2max);
        assignin('base','theta2min',theta2min);
        assignin('base','x',x);
        assignin('base','y',y);
        assignin('base','z',z);
    end
    
elseif get(handles.pop,'Value') == 2 && strcmp(s,'realTimePlotOn') %3D RRP
    cla(handles.axes1) % clear any previous drawing on the axes
    Drawing_Step=pi*str2double(get(handles.draw_step,'string'))/180;; % Assign a fixed drawing step
    Len_Step    =str2double(get(handles.len_step,'string'));          %L1 len step
    l1min=str2double(get(handles.l1Min,'string'));
    l1max=str2double(get(handles.l1Max,'string'));
    theta1min=pi*str2double(get(handles.angel1Min,'string'))/180;
    theta1max=pi*str2double(get(handles.angel1Max,'string'))/180;
    theta2min=pi*str2double(get(handles.angel2Min,'string'))/180;
    theta2max=pi*str2double(get(handles.angel2Max,'string'))/180;
    
    view(handles.axes1,3);  %tell axes1 to view in 3D
    hold on
    for len = l1min:Len_Step:l1max
        for ang1 =theta1min:Drawing_Step:theta1max
            for ang2 =theta2min:Drawing_Step:theta2max
                z = len * cos(ang1);
                x = len * sin(ang1) * cos(ang2);
                y = len * sin(ang1) * sin(ang2);
                plot3(handles.axes1,x,y,z,'*');
                %scatter3(handles.axes1,x,y,z,'*');
            end
            drawnow                                                     %the same idea i used in 2d
        end
    end
end

set(handles.plotting,'Visible','off');





% --- Executes on selection change in pop.
function pop_Callback(hObject, eventdata, handles)
% hObject    handle to pop (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if get(handles.pop,'Value') == 2              %3D ploting
    set(handles.angel2Max,'Visible','on');
    set(handles.text6,'Visible','on');
    set(handles.angel2Min,'Visible','on');
    set(handles.text7,'Visible','on');
    set(handles.rotate,'Visible','on');
    %set(handles.checkbox5,'Visible','on');
    set(handles.slider2,'Visible','on');
    set(handles.slider4,'Visible','on');
    set(handles.slider5,'Visible','on');
    set(handles.text9,'Visible','on');
    set(handles.text10,'Visible','on');
    set(handles.text11,'Visible','on');
else                                         %2d plotting
    set(handles.angel2Max,'Visible','off');
    set(handles.text6,'Visible','off');
    set(handles.angel2Min,'Visible','off');
    set(handles.text7,'Visible','off');
    set(handles.rotate,'Visible','off');
    %set(handles.checkbox5,'Visible','off');
    set(handles.slider2,'Visible','off');
    set(handles.slider4,'Visible','off');
    set(handles.slider5,'Visible','off');
    set(handles.text9,'Visible','off');
    set(handles.text10,'Visible','off');
    set(handles.text11,'Visible','off');
end

% Hints: contents = cellstr(get(hObject,'String')) returns pop contents as cell array
%        contents{get(hObject,'Value')} returns selected item from pop


% --- Executes during object creation, after setting all properties.
function pop_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pop (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over pop.
function pop_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to pop (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



function angel2Max_Callback(hObject, eventdata, handles)
% hObject    handle to angel2Max (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of angel2Max as text
%        str2double(get(hObject,'String')) returns contents of angel2Max as a double


% --- Executes during object creation, after setting all properties.
function angel2Max_CreateFcn(hObject, eventdata, handles)
% hObject    handle to angel2Max (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function angel2Min_Callback(hObject, eventdata, handles)
% hObject    handle to angel2Max (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of angel2Max as text
%        str2double(get(hObject,'String')) returns contents of angel2Max as a double


% --- Executes during object creation, after setting all properties.
function angel2Min_CreateFcn(hObject, eventdata, handles)
% hObject    handle to angel2Max (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function angel3_Callback(hObject, eventdata, handles)
% hObject    handle to angel3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of angel3 as text
%        str2double(get(hObject,'String')) returns contents of angel3 as a double


% --- Executes during object creation, after setting all properties.
function angel3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to angel3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
plot3(handles.axes1,1,1,1,'x');
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



function rot_Callback(hObject, eventdata, handles)
% hObject    handle to rot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of rot as text
%        str2double(get(hObject,'String')) returns contents of rot as a double


% --- Executes during object creation, after setting all properties.
function rot_CreateFcn(hObject, eventdata, handles)
% hObject    handle to rot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%view (handles.axes1,180,90);
%set(handles.axes1,'CameraPosition',[20 10 10]);
clc
p=get(handles.axes1,'CameraPosition')
p.x = 30
get(handles.axes1,'CameraPosition')
handles.axes1
%set(handles.axes1,'CameraPosition',p);


% --- Executes on button press in grid.
function grid_Callback(hObject, eventdata, handles)
% hObject    handle to grid (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if get(hObject,'Value') == 1 && get(handles.pop,'value') == 1    %check box pressed
    grid on
elseif get(hObject,'Value') == 0  && get(handles.pop,'value') == 1
    grid off
end
% Hint: get(hObject,'Value') returns toggle state of grid


% --- Executes on button press in rotate.
function rotate_Callback(hObject, eventdata, handles)
% hObject    handle to rotate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if get(hObject,'Value') == 1
    set(handles.checkbox5,'value',0);
    rotate3d on
elseif get(hObject,'Value') == 0
    set(handles.checkbox5,'value',1);
    rotate3d off
end

% Hint: get(hObject,'Value') returns toggle state of rotate


% --- Executes on button press in realTime.
function realTime_Callback(hObject, eventdata, handles)
% hObject    handle to realTime (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if get(hObject,'Value') == 1   %check box checked
    s = 'realTimePlotOn';
    set(handles.caution,'visible','on');
elseif get(hObject,'Value') == 0
    s = 'realTimePlotOff';
    set(handles.caution,'visible','off');
end
handles.s=s;
guidata(hObject,handles); %update handles to update value of s in it
% Hint: get(hObject,'Value') returns toggle state of realTime


% --- Executes on slider movement.
function slider2_Callback(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
ax = campos(handles.axes1);   %rotate x
x=get(hObject,'Value');%*15;
campos([x,ax(2),ax(3)]);

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
set(hObject,'Max',360);
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes during object creation, after setting all properties.
function axes1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes1


% --- Executes on slider movement.
function slider4_Callback(hObject, eventdata, handles)
% hObject    handle to slider4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
ax = campos(handles.axes1);      %rotate y
y=get(hObject,'Value');%*15;
campos([ax(1),y,ax(3)]);
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
set(hObject,'Max',360);
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider5_Callback(hObject, eventdata, handles)
% hObject    handle to slider5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
ax = campos(handles.axes1);       %rotate z
z=get(hObject,'Value');%*15;
campos([ax(1),ax(2),z]);
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
set(hObject,'Max',360);
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



function draw_step_Callback(hObject, eventdata, handles)
% hObject    handle to draw_step (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of draw_step as text
%        str2double(get(hObject,'String')) returns contents of draw_step as a double


% --- Executes during object creation, after setting all properties.
function draw_step_CreateFcn(hObject, eventdata, handles)
% hObject    handle to draw_step (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function len_step_Callback(hObject, eventdata, handles)
% hObject    handle to len_step (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of len_step as text
%        str2double(get(hObject,'String')) returns contents of len_step as a double


% --- Executes during object creation, after setting all properties.
function len_step_CreateFcn(hObject, eventdata, handles)
% hObject    handle to len_step (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in clc.
function clc_Callback(hObject, eventdata, handles)
% hObject    handle to clc (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
clear variables or clearvars;


% --- Executes on button press in DataInBase.
function DataInBase_Callback(hObject, eventdata, handles)
% hObject    handle to DataInBase (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%handles.showData = get(hObject,'Value');

% Hint: get(hObject,'Value') returns toggle state of DataInBase


% --- Executes on button press in checkbox5.
function checkbox5_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if get(hObject,'Value') == 1
    set(handles.rotate,'value',0);
    datacursormode on;
else
    set(handles.rotate,'value',1);
    datacursormode off;
end

% Hint: get(hObject,'Value') returns toggle state of checkbox5
