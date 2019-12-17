function varargout = PlotCorrMap(varargin)
% PLOTCORRMAP MATLAB code for PlotCorrMap.fig
%      PLOTCORRMAP, by itself, creates a new PLOTCORRMAP or raises the existing
%      singleton*.
%
%      H = PLOTCORRMAP returns the handle to a new PLOTCORRMAP or the handle to
%      the existing singleton*.
%
%      PLOTCORRMAP('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in PLOTCORRMAP.M with the given input arguments.
%
%      PLOTCORRMAP('Property','Value',...) creates a new PLOTCORRMAP or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before PlotCorrMap_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to PlotCorrMap_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help PlotCorrMap

% Last Modified by GUIDE v2.5 16-Dec-2019 10:48:41

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @PlotCorrMap_OpeningFcn, ...
                   'gui_OutputFcn',  @PlotCorrMap_OutputFcn, ...
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


% --- Executes just before PlotCorrMap is made visible.
function PlotCorrMap_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to PlotCorrMap (see VARARGIN)

% Choose default command line output for PlotCorrMap
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

%Get the plotCorrObj from the main GUI (Manuvent_corr)
MC_h = findobj('Tag', 'Manuvent_corr');
MC_data = guidata(MC_h);
MC_passed = get(MC_data.Plot_correlation, 'UserData');
plotCorrObj = MC_passed.plotCorrObj;

%Get the current postition for the seed
curPos = plotCorrObj.curPos;

%Get the flag for whether regressing out background
reg_flag = plotCorrObj.reg_flag;

clear MC_data MC_passed;

%Plot the correlation map related to the seed
corrM = plotCorrelationMap(plotCorrObj, reg_flag, handles);
%handles.Status.Visible = 'On';  handles.Status.String = 'Finished!';

%Calculate the averaged correlation near the seed
avg_seed_corr = calculateSurrounding(curPos, corrM);
handles.edit_seed.String = num2str(avg_seed_corr);
handles.Save_data.UserData.avg_seed_corr = avg_seed_corr;

%Initiate Save_data.UserData
handles.Save_data.UserData.avg_rec_corr = [];
handles.Save_data.UserData.avg_hand_corr = [];
handles.Save_data.UserData.rec_Position = [];
handles.Save_data.UserData.hand_Position = [];

%Save the correlation matrix
handles.output.UserData.corrM = corrM;
handles.output.UserData.plotCorrObj = plotCorrObj;

%Set default percentile value
handles.Corr_percentile.UserData.curValue = 0.997;
handles.Corr_percentile.String = num2str(99.7);
disp('Default percentile for highly correlated region = 99.7%.')

%Set the status text
handles.Status.Visible = 'On';
handles.Status.String = plotCorrObj.filename;

% UIWAIT makes PlotCorrMap wait for user response (see UIRESUME)
% uiwait(handles.figure1);


function corrM = plotCorrelationMap(plotCorrObj, reg_flag, handles)
% Plot the correlation map w.r.t the current seed
    A = plotCorrObj.curMovie;
    curPos = plotCorrObj.curPos;
    
    %Get the fluorescent trace of the current roi
    seedTrace = A(curPos(2), curPos(1), :);
    seedTrace = seedTrace(:);
    sz  = size(A);
    imgall = reshape(A, sz(1)*sz(2), sz(3));
    
    %Calculate correlation matrix
    if ~reg_flag
        corrM = corr(imgall',seedTrace);
    else
        std_all = nanstd(imgall,0,2);
        lowPixels = std_all <= prctile(std_all,5);
        %avg_trace = nanmean(imgall,1);
        avg_trace = nanmean(imgall(lowPixels,:),1);
        corrM = partialcorr(imgall',seedTrace, avg_trace');
    end
    corrM = reshape(corrM, sz(1:2));
    
    %Plot the correlation map
    hold off;
    im = imagesc(handles.CorrMap, corrM); colormap jet;
    colorbar(handles.CorrMap); 
    axis(handles.CorrMap, 'image');
    caxis(handles.CorrMap,[-0.2, 1]); 
    title(handles.CorrMap,['roi:', num2str(curPos)]);
    
    set(im, 'ButtonDownFcn', {@markEvents, handles});

    hold(handles.CorrMap, 'on')
    
    %Label the seed
    x1 = curPos(1);
    y1 = curPos(2);
    fill(handles.CorrMap,[x1-2,x1-2,x1+2,x1+2],[y1-2,y1+2,y1+2,y1-2], 'y')

    
function [y2 ,x2] = findReccomandMax(corrM)
%Find the point that shows maximum correlation other than the seed given
%certain conditions

%Filter to mask out boundary artifact (due to rigid registration)
filter = ones(5);
corrM_conv = conv2(corrM,filter,'same');

%Find the maximum point
max_point = max(corrM_conv(:));
[y2 ,x2] = find(corrM_conv == max_point);
fill([x2-2,x2-2,x2+2,x2+2],[y2-2,y2+2,y2+2,y2-2], 'm')

function avg_corr = calculateSurrounding(curPos, corrM)
    x = curPos(1);
    y = curPos(2);
    surrond = corrM(y-2:y+2,x-2:x+2);
    avg_corr = nanmean(surrond(:));

function markEvents(h,~,handles)
%This function will allow user to mark a new event as well as store
%information of the defined roi (including the postition, the frame indices
%when the roi/event was created/initiated and deleted/ended)
%h        handle of the current image
%handles  handles of the GUI
roi = drawpoint(h.Parent, 'Color', 'g'); %Drawing a new roi
curPos = round(roi.Position);  %Current xy coordinates
corrM = handles.output.UserData.corrM;
avg_hand_corr = calculateSurrounding(curPos, corrM);
handles.edit_hand.String = num2str(avg_hand_corr);
handles.Save_data.UserData.avg_hand_corr = avg_hand_corr;
handles.Save_data.UserData.hand_Position = curPos;

%Listening to the moving events
addlistener(roi, 'ROIMoved', @(src,evt)movedCallback(src,evt,handles));


function movedCallback(roi,~,handles)
%Actions to take when moved the roi
%roi     the Point obj
%handles     handles of the GUI
curPos = round(roi.Position);  %Current xy coordinates
corrM = handles.output.UserData.corrM;
avg_hand_corr = calculateSurrounding(curPos, corrM);
handles.edit_hand.String = num2str(avg_hand_corr);
handles.Save_data.UserData.avg_hand_corr = avg_hand_corr;
handles.Save_data.UserData.hand_Position = curPos;


% --- Outputs from this function are returned to the command line.
function varargout = PlotCorrMap_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function edit_seed_Callback(hObject, eventdata, handles)
% hObject    handle to edit_seed (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_seed as text
%        str2double(get(hObject,'String')) returns contents of edit_seed as a double


% --- Executes during object creation, after setting all properties.
function edit_seed_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_seed (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_rec_Callback(hObject, eventdata, handles)
% hObject    handle to edit_rec (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_rec as text
%        str2double(get(hObject,'String')) returns contents of edit_rec as a double


% --- Executes during object creation, after setting all properties.
function edit_rec_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_rec (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_hand_Callback(hObject, eventdata, handles)
% hObject    handle to edit_hand (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_hand as text
%        str2double(get(hObject,'String')) returns contents of edit_hand as a double


% --- Executes during object creation, after setting all properties.
function edit_hand_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_hand (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in Save_data.
function Save_data_Callback(hObject, eventdata, handles)
% hObject    handle to Save_data (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
try
    plotCorrObj = handles.output.UserData.plotCorrObj;
    reg_flag = plotCorrObj.reg_flag;
    curPos = plotCorrObj.curPos;
    saveas(handles.CorrMap, ['roi_', num2str(reg_flag), '_',...
        num2str(curPos(1)) '_',  num2str(curPos(2)),'.png'])  
    avg_seed_corr = handles.Save_data.UserData.avg_seed_corr;
    avg_rec_corr = handles.Save_data.UserData.avg_rec_corr;
    avg_hand_corr = handles.Save_data.UserData.avg_hand_corr;
    rec_Position = handles.Save_data.UserData.rec_Position;
    hand_Position = handles.Save_data.UserData.hand_Position;
    save(['roi_', num2str(reg_flag), '_',...
        num2str(curPos(1)) '_',  num2str(curPos(2)),'.mat'],...
        'avg_seed_corr', 'avg_rec_corr', 'avg_hand_corr', 'curPos',...
        'rec_Position', 'hand_Position');
catch
    warning('Variables not defined!')
end


% --- Executes on button press in Draw_region.
function Draw_region_Callback(hObject, eventdata, handles)
% hObject    handle to Draw_region (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.Status.Visible = 'On';
handles.Status.String = 'Pls draw a region!';
BW = roipoly;
corrM = handles.output.UserData.corrM;
corrM_masked = corrM .* BW;
handles.Draw_region.UserData.corrM_masked = corrM_masked;
handles.Status.String = 'ROI defined!';


% --- Executes on button press in Find_max.
function Find_max_Callback(hObject, eventdata, handles)
% hObject    handle to Find_max (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
try
    corrM_masked = handles.Draw_region.UserData.corrM_masked;
    %Find the point that shows maximum correlation in a defined roi region
    [y2 ,x2] = findReccomandMax(corrM_masked);
    avg_rec_corr = calculateSurrounding([x2 y2], corrM_masked);
    handles.edit_rec.String = num2str(avg_rec_corr);
    handles.Save_data.UserData.avg_rec_corr = avg_rec_corr;
    handles.Save_data.UserData.rec_Position = [x2, y2];
    handles.Status.Visible = 'On';
    handles.Status.String = 'Maximum found!';
catch
    msgbox('Please define a roi region first!', 'Error');
end


% --- Executes during object creation, after setting all properties.
function Save_data_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Save_data (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes on button press in Corr_region.
function Corr_region_Callback(hObject, eventdata, handles)
% hObject    handle to Corr_region (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

try
    %Renew the map 
    plotCorrObj = handles.output.UserData.plotCorrObj;
    reg_flag = plotCorrObj.reg_flag;
    plotCorrelationMap(plotCorrObj, reg_flag, handles);
    
    %Plot the highly correlated region
    curThreshold = handles.Corr_percentile.UserData.curValue;
    corrM = handles.output.UserData.corrM;
    [x,y] = find(corrM>=curThreshold);
    Correlated_region.x = x;
    Correlated_region.y = y;
    hObject.UserData.Correlated_region = Correlated_region;
    plot(handles.CorrMap, y,x,'w*')  
catch
    msgbox('Can not get the correlated region!', 'Error!')
    return
end


function Corr_percentile_Callback(hObject, eventdata, handles)
% hObject    handle to Corr_percentile (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Corr_percentile as text
%        str2double(get(hObject,'String')) returns contents of Corr_percentile as a double
try
    %Get and calculate the percentile
    curPrct = str2double(get(hObject, 'String'));
    handles.Corr_percentile.UserData.curValue = curPrct/100;
    disp(['Plot region with correlation > ' num2str(curPrct) '%'])
catch
    msgbox('Please input a number btw 0-100!', 'Error!')
end

% --- Executes during object creation, after setting all properties.
function Corr_percentile_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Corr_percentile (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in Plot_avg.
function Plot_avg_Callback(hObject, eventdata, handles)
% hObject    handle to Plot_avg (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


try
    %Get required data
    curThreshold = handles.Corr_percentile.UserData.curValue;
    corrM = handles.output.UserData.corrM;
    plotCorrObj = handles.output.UserData.plotCorrObj;
    curMovie = plotCorrObj.curMovie;
    
    %Reshape matrices
    sz = size(corrM);
    corrM = reshape(corrM, [sz(1)*sz(2),1]);
    curMovie = reshape(curMovie, [sz(1)*sz(2), size(curMovie,3)]);
    
    %Calculate the averaged trace
    Avg_trace = nanmean(curMovie(corrM>=curThreshold,:),1);
    hObject.UserData.Avg_trace = Avg_trace;
    
    %Plot the trace
    plot(handles.Trace, Avg_trace, 'LineWidth', 2);
    max_time = find(Avg_trace == max(Avg_trace));
    hold(handles.Trace, 'on');
    plot(max_time, max(Avg_trace), 'r*')
    plot(10:14, Avg_trace(10:14), 'g', 'LineWidth', 2)
    Duration = length(Avg_trace);
    plot(1:Duration, 0.03*ones(Duration,1), 'r')
    
    %Save the values to UserData
    hObject.UserData.max_time = max_time;
    hObject.UserData.max_value = max(Avg_trace);
    
catch
    msgbox('Can not plot the averaged trace!', 'Error!')
    return
end


% --- Executes on button press in Save_trace.
function Save_trace_Callback(hObject, eventdata, handles)
% hObject    handle to Save_trace (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

curThreshold = handles.Corr_percentile.UserData.curValue;
Correlated_region = handles.Corr_region.UserData.Correlated_region;
Avg_trace = handles.Plot_avg.UserData.Avg_trace;
max_time = handles.Plot_avg.UserData.max_time;
max_value = handles.Plot_avg.UserData.max_value;
plotCorrObj = handles.output.UserData.plotCorrObj;
filename = plotCorrObj.filename;

%Locate specific tag
t = strfind(filename,'AveragedMatrix');
if ~isempty(t)
    filename = filename(t+15:end);
end

savename = [filename '_averaged_trace_' num2str(curThreshold)];
save([savename '.mat'], 'curThreshold', 'Correlated_region', 'Avg_trace', 'max_time', 'max_value')
saveas(handles.Trace, [savename '.png'])  
