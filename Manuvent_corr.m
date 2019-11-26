function varargout = Manuvent_corr(varargin)
% MANUVENT_CORR MATLAB code for Manuvent_corr.fig
%      MANUVENT_CORR, by itself, creates a new MANUVENT_CORR or raises the existing
%      singleton*.
%
%      H = MANUVENT_CORR returns the handle to a new MANUVENT_CORR or the handle to
%      the existing singleton*.
%
%      MANUVENT_CORR('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in MANUVENT_CORR.M with the given input arguments.
%
%      MANUVENT_CORR('Property','Value',...) creates a new MANUVENT_CORR or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Manuvent_corr_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Manuvent_corr_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Manuvent_corr

% Last Modified by GUIDE v2.5 25-Nov-2019 23:34:12

% Version 0.0.2 11/01/2019 yixiang.wang@yale.edu

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Manuvent_corr_OpeningFcn, ...
                   'gui_OutputFcn',  @Manuvent_corr_OutputFcn, ...
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

% --- Executes just before Manuvent_corr is made visible.
function Manuvent_corr_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Manuvent_corr (see VARARGIN)

% Choose default command line output for Manuvent_corr
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes Manuvent_corr wait for user response (see UIRESUME)
% uiwait(handles.Manuvent_corr);


% --- Outputs from this function are returned to the command line.
function varargout = Manuvent_corr_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in Load_movie.
function Load_movie_Callback(hObject, eventdata, handles)
% hObject    handle to Load_movie (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%Detect whether data acuqired from the last run was saved
if isfield(hObject.UserData, 'filename')&& ~isempty(handles.listbox.UserData.allROI_info)
    filename = hObject.UserData.filename;
    savename = [filename(1:end-4) '_allROI_info.mat'];
    if ~exist(savename,'file')
        selection = questdlg('Did you save the previous data?','Confirm save','Save','Yes, I did.','Save');
        switch selection
            case 'Save'
                allROI_info = handles.listbox.UserData.allROI_info;
                allROI = handles.listbox.UserData.allROI;
                %save all ROI(events) info as a .mat file
                uisave({'allROI_info', 'allROI'}, [filename(1:end-4) '_allROI.mat']);
        end
    end
end

%Clean previous data if existed
handles.listbox.String = {};
handles.listbox.UserData.allROI = {};
handles.listbox.UserData.allROI_info = [];
handles.listbox.Value = 1;

%Show loading progress
set(handles.Text_load, 'Visible', 'On')
set(handles.Text_load, 'String', 'Loading...')
set(handles.Text_playing, 'Visible', 'off')

%Choose the folder where filtered matriices are saved
selpath = uigetdir('Please choose the folder where filtered matrices are!');
cd(selpath);
fileList = dir('*filtered*');
A_all = [];

for i = 1:size(fileList,1)
    curLoad = load(fileList(i).name);
    try
        A_dFoF = curLoad.A_dFoF;
        A_all = cat(3, A_all, A_dFoF);
        disp(['Loaded ' num2str(i) 'th matrix.'])
        clear A_dFoF
    catch
        disp(['This file ' fileList(i).name ' does not contain movie matrix!'])
    end
end

handles.Load_movie.UserData.filename = fileList(1).name;

%Clean index information from the previous movie
handles.play.UserData = [];
handles.Movie_control.UserData.curIdx = 1;
handles.Frame.String = '1';

%Store the movie into a variable curMovie
try
    curMovie = A_all;
    set(handles.Text_load, 'String', 'Finished!')
 
catch
    set(handles.Text_load, 'String', 'Error!')
    warning('Can not load the dF over F movie!')        
end

%Save the movie and its size as an object to the UserData of the GUI
sz = size(curMovie);
curObj.sz = sz;
curObj.duration = sz(3);
curObj.curMovie = curMovie;
set(handles.output, 'UserData', curObj);

%Set parameters for slider1
set(handles.slider1, 'Min', 1);
set(handles.slider1, 'Max', sz(3));
set(handles.slider1, 'Value', 1);
set(handles.slider1, 'SliderStep', [1/(sz(3)-1), 0.05]);

hold off;
im = imshow(mat2gray(curMovie(:,:,1)), 'Parent', handles.axes1);
set(im, 'ButtonDownFcn', {@markEvents, handles});

% --- Executes on button press in Save_data.
function Save_data_Callback(hObject, eventdata, handles)
% hObject    handle to Save_data (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
filename = handles.Load_movie.UserData.filename;
allROI_info = handles.listbox.UserData.allROI_info;
allROI = handles.listbox.UserData.allROI;
%save all ROI(events) info as a .mat file
uisave({'allROI_info', 'allROI'}, [filename(1:end-4) '_allROI.mat']);


% --- Executes on selection change in listbox1.
function listbox1_Callback(hObject, eventdata, handles)
% hObject    handle to listbox1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns listbox1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from listbox1


% --- Executes during object creation, after setting all properties.
function listbox1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to listbox1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



% --- Executes on selection change in listbox.
function listbox_Callback(hObject, eventdata, handles)
% hObject    handle to listbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns listbox contents as cell array
%        contents{get(hObject,'Value')} returns selected item from listbox

curMovie = handles.output.UserData.curMovie;%Get current movie
curVal = get(hObject,'Value'); %Get current value
allROI = handles.listbox.UserData.allROI; %Get all ROI objects
allROI_info = handles.listbox.UserData.allROI_info;
roi = allROI{curVal}; %Get corresponding roi obj
roi_info = allROI_info(curVal,:); %Get corresponding roi info

%Get the index of the first frame
ini_idx = roi_info(3); 

%Show current roi
roi.Parent = handles.axes1;
roi.Visible = 'on';
roi.Color = 'r';
pause(0.5);
roi.Color = 'g';

%Listening to the deleting events
addlistener(roi, 'DeletingROI', @(src,evt)deleteCallback(src,evt,handles));
%Listening to the moving events
addlistener(roi, 'ROIMoved', @(src,evt)movedCallback(src,evt,handles));
%Listening to the clicking events
addlistener(roi, 'ROIClicked', @(src,evt)clickedCallback(src,evt,handles));

hold on;

%Update current roi
handles.Plot_correlation.UserData.curROI = roi;

%Jump to the frame where the current roi was created
im = imshow(mat2gray(curMovie(:,:,ini_idx)), 'Parent', handles.axes1);
set(im, 'ButtonDownFcn', {@markEvents, handles});
%Set current index to the initial index of the selected event
handles.Movie_control.UserData.curIdx = ini_idx; 
%Reset slider value
handles.slider1.Value = ini_idx;
%Reset Frame editbox string
handles.Frame.String = num2str(ini_idx);

set(handles.Text_playing, 'String', 'First frame')


% --- Executes during object creation, after setting all properties.
function listbox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to listbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
hObject.String = {};
hObject.UserData.allROI = {};
hObject.UserData.allROI_info = [];

% --- Executes on slider movement.
function slider1_Callback(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
curMovie = handles.output.UserData.curMovie; %Get current movie
curIdx = round(get(hObject, 'Value'));
im = imshow(mat2gray(curMovie(:,:,curIdx)), 'Parent', handles.axes1);
set(im, 'ButtonDownFcn', {@markEvents, handles});
handles.Movie_control.UserData.curIdx = curIdx;
set(handles.Frame, 'String', num2str(curIdx));


% --- Executes during object creation, after setting all properties.
function slider1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in FastBackward.
function FastBackward_Callback(hObject, eventdata, handles)
% hObject    handle to FastBackward (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

curMovie = handles.output.UserData.curMovie;
curIdx = handles.Movie_control.UserData.curIdx;

if (curIdx - 10) < 1
    curIdx = 1;
else
    curIdx = curIdx - 10; %Fast backward 10 frames
end
handles.Movie_control.UserData.curIdx = curIdx;
set(handles.slider1, 'Value', curIdx);
set(handles.Frame, 'String', num2str(curIdx));
im = imshow(mat2gray(curMovie(:,:,curIdx)), 'Parent', handles.axes1); %Display current frame
set(im, 'ButtonDownFcn', {@markEvents, handles});


% --- Executes on button press in FastForward.
function FastForward_Callback(hObject, eventdata, handles)
% hObject    handle to FastForward (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

duration = handles.output.UserData.duration;
curMovie = handles.output.UserData.curMovie;
curIdx = handles.Movie_control.UserData.curIdx;

if (curIdx + 10) > duration
    curIdx = duration;
else
    curIdx = curIdx + 10;
end
handles.Movie_control.UserData.curIdx = curIdx;
set(handles.slider1, 'Value', curIdx);
set(handles.Frame, 'String', num2str(curIdx));
im = imshow(mat2gray(curMovie(:,:,curIdx)), 'Parent', handles.axes1); %Display current frame
set(im, 'ButtonDownFcn', {@markEvents, handles});


% --- Executes on button press in play.
function play_Callback(hObject, eventdata, handles)
% hObject    handle to play (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

try
    %whether a new movie has been loaded
    if isempty(hObject.UserData)
        curFlag = 0; 
        hObject.UserData.IsFirstCall = 1; %First time call the callback
    else
        curFlag = ~hObject.UserData.curFlag; %Renew current flag
    end
    
    hObject.UserData.curFlag = curFlag; %Update/store the renewed flag 
    curIdx = handles.Movie_control.UserData.curIdx; %Get current index
    curObj = handles.output.UserData; 
    curMovie = curObj.curMovie; %Get current movie
    duration = handles.output.UserData.duration;%Get movie duration

    if ~curFlag
        hObject.String = 'Pause';
    elseif curFlag 
        hObject.String = 'Play';
    end

    while ~curFlag %If current action was acting on 'Play'        
        if hObject.UserData.curFlag && ~hObject.UserData.IsFirstCall
            %When callback if the previous action was 'Pause' and if it is
            %not the first time the callback function being called
            handles.Movie_control.UserData.curIdx = curIdx; %Update/store current frame index
            break
        end        
        
        if curIdx > duration
            hObject.String = 'Play';
            curFlag = 1;
            hObject.UserData.curFlag = curFlag; %Update/store the renewed flag 
            break;
        end
        
        set(handles.Frame, 'String', num2str(curIdx));
        set(handles.slider1, 'Value', curIdx);
        handles.Movie_control.UserData.curIdx = curIdx; %Update/store current frame index
        imshow(mat2gray(curMovie(:,:,curIdx)), 'Parent', handles.axes1);
        curIdx = curIdx + 1; %Movie to the next frame
        pause(0.1);
        
    end
    
    %[x, y] = getpts(handles.axes1);
    %plot(handles.axes1, x, y, 'ro')
    %roi = drawpoint(handles.axes1);
    im = imshow(mat2gray(curMovie(:,:,curIdx)), 'Parent', handles.axes1); %Display current frame
    set(im, 'ButtonDownFcn', {@markEvents, handles});
    hObject.UserData.IsFirstCall = 0; %Now it's not the first time the callback being called
    
catch
    
    if isempty(handles.output.UserData)
        error('Please load a movie first!')
    end
end


function markEvents(h,~,handles)
%This function will allow user to mark a new event as well as store
%information of the defined roi (including the postition, the frame indices
%when the roi/event was created/initiated and deleted/ended)
%h        handle of the current image
%handles  handles of the GUI
    
    allROI = handles.listbox.UserData.allROI;
    allROI_info = handles.listbox.UserData.allROI_info; %Get all ROIs' inforamtion
    curList = handles.listbox.String; %Get display string from listbox
    curIdx = handles.Movie_control.UserData.curIdx; %Get current frame index
    
    roi = drawpoint(h.Parent, 'Color', 'g'); %Drawing a new roi
    curPos = round(roi.Position);  %Current xy coordinates
    curStr = [num2str(curIdx) ' ' num2str(curPos(1)) ' ' num2str(curPos(2))]; %New string to be listed in listbox
    curList{end+1} = curStr; %Add new string to string cell array
    handles.listbox.String = curList; %Renew listbox (display new string)
    roi.UserData.Str = curStr; %Attach string information to the new roi
    roi.UserData.Idx = curIdx; %Attach index information to the new roi
    
    roiInfo = [curPos, curIdx, curIdx]; %New roi's info
    if isempty(allROI_info)
        allROI_info = roiInfo;
    else
        allROI_info(end+1,:) = roiInfo; %Add new roi's info to current info list
    end
    allROI{end+1} = roi; %Add new roi obj to current roi array
    handles.listbox.UserData.allROI_info = allROI_info; %Store renewed info list
    handles.listbox.UserData.allROI = allROI; %Store renewed roi array
    
    %Listening to the deleting events
    addlistener(roi, 'DeletingROI', @(src,evt)deleteCallback(src,evt,handles));
    %Listening to the moving events
    addlistener(roi, 'ROIMoved', @(src,evt)movedCallback(src,evt,handles));
    %Listening to the clicking events
    addlistener(roi, 'ROIClicked', @(src,evt)clickedCallback(src,evt,handles));
    %Hold the current position
    hold on
    
    %Update current roi
    handles.Plot_correlation.UserData.curROI = roi;
    
    function deleteCallback(roi,~,handles)
    %Actions to take when deleting the roi
    %roi     the Point obj
    %handles     handles of the GUI
    
        %Get the frame index when the delte action is called
        %curIdx = handles.Movie_control.UserData.curIdx;
        %Get the frame index when the roi was created
        %iniIdx = roi.UserData.Idx;
        
        %Get the current string array from the listbox
        curList = handles.listbox.String;
        allROI_info = handles.listbox.UserData.allROI_info; %Get all ROIs' inforamtion
        allROI = handles.listbox.UserData.allROI; %Get all roi objects
                  
        allROI_info(strcmp(curList,roi.UserData.Str),:) = []; %Delete the roi info
        allROI = allROI(~strcmp(curList,roi.UserData.Str)); %Delete the roi obj
        handles.listbox.UserData.allROI_info = allROI_info; %Store renewed info list
        handles.listbox.UserData.allROI = allROI; %Store renewed roi array

        curList = curList(~strcmp(curList,roi.UserData.Str)); %Delete the corresponding string
        handles.listbox.String = curList; %Renew the display

        
    function movedCallback(roi,~,handles)
    %Actions to take when moved the roi
    %roi     the Point obj
    %handles     handles of the GUI

        curPos = round(roi.Position);  %Current xy coordinates

        %Get the current string array from the listbox
        curList = handles.listbox.String;
        allROI_info = handles.listbox.UserData.allROI_info; %Get all ROIs' inforamtion
        allROI = handles.listbox.UserData.allROI; %Get all roi objects
        allROI_info(strcmp(curList,roi.UserData.Str),1:2) = curPos; %Renew the xy coordinates
        allROI(strcmp(curList,roi.UserData.Str)) = {roi}; %Renew the roi
        handles.listbox.UserData.allROI_info = allROI_info; %Store renewed info list
        handles.listbox.UserData.allROI = allROI; %Store renew obj array

        %New string to be listed in listbox
        curStr = [num2str(roi.UserData.Idx) ' ' num2str(curPos(1)) ' ' num2str(curPos(2))];            

        curList(strcmp(curList,roi.UserData.Str)) = {curStr}; %Renew the displayed position
        handles.listbox.String = curList; %Renew the display

        roi.UserData.Str = curStr; %Renew the string info of the roi obj
        %plot(h.Parent, xy(1), xy(2), 'ro')
       
    function clickedCallback(roi,evt,handles)
    %Actions to take when clicked the roi
    %roi     the Point obj
    %evt     current event
    %handles     handles of the GUI

        if strcmp(evt.SelectionType,'double')
            %Get the frame index when the delte action is called
            curIdx = handles.Movie_control.UserData.curIdx;
            %Get the frame index when the roi was created
            %iniIdx = roi.UserData.Idx;

            %Get the current string array from the listbox
            curList = handles.listbox.String;
            allROI_info = handles.listbox.UserData.allROI_info; %Get all ROIs' inforamtion

            %Store the current frame index (end of an event) to the roi
            allROI_info(strcmp(curList,roi.UserData.Str),4) = curIdx;
            handles.listbox.UserData.allROI_info = allROI_info; %Store the renewed info list

            %Make the current roi invisible
            roi.Visible = 'off';

        elseif strcmp(evt.SelectionType,'left')
            if roi.Color(2) == 1
                roi.Color = 'r';
            else
                roi.Color = 'g';
            end
        end



% --- Executes on button press in Stop.
function Stop_Callback(hObject, eventdata, handles)
% hObject    handle to Stop (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

curIdx = 1; %Reset to the first frame
curMovie = handles.output.UserData.curMovie;
handles.Movie_control.UserData.curIdx = curIdx;
set(handles.slider1, 'Value', curIdx);
set(handles.Frame, 'String', num2str(curIdx));
im = imshow(mat2gray(curMovie(:,:,curIdx)), 'Parent', handles.axes1); %Display current frame
set(im, 'ButtonDownFcn', {@markEvents, handles});


function Text_load_Callback(hObject, eventdata, handles)
% hObject    handle to Text_load (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Text_load as text
%        str2double(get(hObject,'String')) returns contents of Text_load as a double


% --- Executes during object creation, after setting all properties.
function Text_load_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Text_load (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function play_CreateFcn(hObject, eventdata, handles)
% hObject    handle to play (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes during object creation, after setting all properties.
function Movie_control_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Movie_control (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
hObject.UserData.curIdx = 1;



function Frame_Callback(hObject, eventdata, handles)
% hObject    handle to Frame (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Frame as text
%        str2double(get(hObject,'String')) returns contents of Frame as a double

try
    curIdx = str2double(get(hObject, 'String'));
    handles.Movie_control.UserData.curIdx = curIdx;
    curMovie = handles.output.UserData.curMovie;
    im = imshow(mat2gray(curMovie(:,:,curIdx)), 'Parent', handles.axes1);
    set(im, 'ButtonDownFcn', {@markEvents, handles});
    set(handles.slider1, 'Value', curIdx);
catch
    error('Exceed movie range!')
end


% --- Executes during object creation, after setting all properties.
function Frame_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Frame (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over listbox.
function listbox_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to listbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on key press with focus on listbox and none of its controls.
function listbox_KeyPressFcn(hObject, eventdata, handles)
% hObject    handle to listbox (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.CONTROL.UICONTROL)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in Load_data.
function Load_data_Callback(hObject, eventdata, handles)
% hObject    handle to Load_data (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Load_movie_Callback(hObject, eventdata, handles)

sz = handles.output.UserData.sz;

%Choose the folder where _allROI.mat files are saved
selpath = uigetdir('Please choose the folder where the ROI files are!');
cd(selpath);
fileList = dir('*allROI*');
combineROI = {};
combineROI_info = [];
for i = 1:size(fileList,1)
    load(fileList(i).name);
    combineROI = [combineROI allROI];
    combineROI_info = [combineROI_info; allROI_info];
end

%Save combined data
allROI = combineROI;
allROI_info = combineROI_info;
filename = fileList(1).name;
filename = [filename(1: strfind(filename,'allROI')-1) '_combined.mat'];
save(filename, 'allROI', 'allROI_info')

if sz(3) >= max(combineROI_info(:,4))
    disp('Sanity check passed...Maximum frame of ROIs does not exceed maximum movie frame.')
else
    msgbox('Detect mismatch between the movie and the ROI file!','Error');
end

%Update UserData
handles.listbox.UserData.allROI_info = allROI_info;
handles.listbox.UserData.allROI = allROI;

importedList = {};
for i = 1:size(allROI,2)
    rounded_str = num2str(round(str2num(allROI{i}.UserData.Str)));
    allROI{i}.UserData.Str = rounded_str;
    importedList{i,1} = rounded_str;
end

handles.listbox.String = importedList;

disp('')



function Text_playing_Callback(hObject, eventdata, handles)
% hObject    handle to Text_playing (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Text_playing as text
%        str2double(get(hObject,'String')) returns contents of Text_playing as a double


% --- Executes during object creation, after setting all properties.
function Text_playing_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Text_playing (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in Replay.
function Replay_Callback(hObject, eventdata, handles)
% hObject    handle to Replay (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

curMovie = handles.output.UserData.curMovie;%Get current movie
curVal = handles.listbox.Value; %Get current value
allROI_info = handles.listbox.UserData.allROI_info; %Get all ROIs' inforamtion
allROI = handles.listbox.UserData.allROI; %Get all ROI objects
roi_info = allROI_info(curVal,:); %Get corresponding roi info

%Play the current event
ini_idx = roi_info(3); 
end_idx = roi_info(4);
%Show event progress
set(handles.Text_playing, 'Visible', 'On')
set(handles.Text_playing, 'String', 'Replaying...')
hObject.Enable = 'off';
for i = ini_idx:end_idx
    imshow(mat2gray(curMovie(:,:,i)), 'Parent', handles.axes1);
    pause(0.05)
end
set(handles.Text_playing, 'String', 'Last frame')
pause(1)
hObject.Enable = 'on';

%Jump to the frame where the current roi was created
im = imshow(mat2gray(curMovie(:,:,ini_idx)), 'Parent', handles.axes1);
set(im, 'ButtonDownFcn', {@markEvents, handles});
%Set current index to the initial index of the selected event
handles.Movie_control.UserData.curIdx = ini_idx; 
%Reset slider value
handles.slider1.Value = ini_idx;
%Reset Frame editbox string
handles.Frame.String = num2str(ini_idx);

set(handles.Text_playing, 'String', 'First frame')



% --- Executes on button press in Hide_pts.
function Hide_pts_Callback(hObject, eventdata, handles)
% hObject    handle to Hide_pts (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
allROI = handles.listbox.UserData.allROI; %Get all ROI objects
%Turn off all ROIs' visibility
for i = 1:length(allROI)
    curPt = allROI{i};
    curPt.Visible = 'off';
end


% --- Executes on button press in Label_movie.
function Label_movie_Callback(hObject, eventdata, handles)
% hObject    handle to Label_movie (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

set(handles.Text_playing, 'Visible', 'On')
set(handles.Text_playing, 'String', 'Labeling...')
disp('Running label movie function...')

%Try to load the movie and ROI info
try
    curMovie = handles.output.UserData.curMovie; %Get current movie
    allROI_info = handles.listbox.UserData.allROI_info; %Get all ROI info
catch
    msgbox('Please load a movie and its ROIs first!','Error');
    return
end

sz = size(curMovie);

try

    %Construct each frame
    parfor i = 1:sz(3)
        %Get the list of ROIs appear on the current frame
        showList = (allROI_info(:,3) <= i).*(allROI_info(:,4) >= i);
        currCentroids = allROI_info(showList>0, 1:2);       
        h = figure('visible','off');
        imshow(mat2gray(curMovie(:,:,i)));
        hold on
        plot(currCentroids(:,1),currCentroids(:,2),'ro','MarkerSize',5,'LineWidth',2)
        hold off
        F(i) = getframe(h);
        close(h)   
        if mod(i,100) == 0
            disp(num2str(i));
        end
    end

    %Create output labeled movie name
    OutputName = 'Labeled_movie.avi';

    % create the video writer with 25 fps
    writerObj = VideoWriter(OutputName);
    writerObj.FrameRate = 25;
    % set the seconds per image
    % open the video writer
    open(writerObj);
    % write the frames to the video
    for i=1:length(F)
        % convert the image to a frame
        frame = F(i);    
        writeVideo(writerObj, frame);
    end
    % close the writer object
    close(writerObj);
    
catch
    warning('Error happened! Probably the movie mismatches the ROIs!')
end

set(handles.Text_playing, 'Visible', 'Off')


% --- Executes on button press in Plot_correlation.
function Plot_correlation_Callback(hObject, eventdata, handles)
% hObject    handle to Plot_correlation (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if ~isempty(handles.Plot_correlation.UserData)
    curRoi = handles.Plot_correlation.UserData.curROI;
    plotCorrObj.curPos = round(curRoi.Position);
    plotCorrObj.curMovie = handles.output.UserData.curMovie;
    
else
    msgbox('Can not detect current roi object!', 'Error')
end

plotCorrObj.reg_flag = handles.regress_flag.Value;
handles.Plot_correlation.UserData.plotCorrObj = plotCorrObj;

disp('Generating the correlation map...')
PlotCorrMap('hObject');


% --- Executes on button press in regress_flag.
function regress_flag_Callback(hObject, eventdata, handles)
% hObject    handle to regress_flag (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of regress_flag
