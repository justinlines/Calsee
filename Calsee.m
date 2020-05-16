%Justin Lines 2015 calsee imaging analysis program

function varargout = calsee(varargin)
% CALSEE MATLAB code for calsee.fig
%      CALSEE, by itself, creates a new CALSEE or raises the existing
%      singleton*.
%
%      H = CALSEE returns the handle to a new CALSEE or the handle to
%      the existing singleton*.
%
%      CALSEE('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in CALSEE.M with the given input arguments.
%
%      CALSEE('Property','Value',...) creates a new CALSEE or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before calsee_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to calsee_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help calsee

% Last Modified by GUIDE v2.5 20-Jan-2020 12:54:11

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @calsee_OpeningFcn, ...
    'gui_OutputFcn',  @calsee_OutputFcn, ...
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


% --- Executes just before calsee is made visible.
function calsee_OpeningFcn(hObject, eventdata, handles, varargin)

handles.output = hObject;
guidata(hObject, handles);


    %% --- INITIALIZE --- %%
function varargout = calsee_OutputFcn(hObject, eventdata, handles)

varargout{1} = handles.output;
axes(handles.axes1)
image(1)
axis off;
axis square;
title('')
set(handles.text1,'String','Please Load a File')
set(handles.text2,'String','0')
set(handles.text4,'String','0')
axes(handles.axes2)
image(1)
axis off;
handles.clusterspace=128; % initialize minimum cluster size
handles.spatialFilter=3; % initialize median filter size
set(handles.popupmenu2,'Value',5); %minimum cluster size display
set(handles.edit2,'String','10'); %baseline constant
set(handles.edit3,'String','2'); %stdThreshold threshold
set(handles.edit4,'String','2'); %StDev trace threshold
set(handles.edit5,'String','10'); %Event detection moving window
set(handles.popupmenu12,'Value',2); %SpatialFilter threshold
handles.baseline=10;
handles.soma_threshold=0.5;
handles.process_threshold=0;
set(handles.popupmenu5,'Value',6); %soma threshold
set(handles.popupmenu6,'Value',1); %process threshold
handles.spike_threshold=2;      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

handles.temporalFilter=3;
set(handles.popupmenu8,'Value',2); %Temporal filtering parameter
handles.fps=5;
set(handles.popupmenu11,'Value',5); %FPS of GIF animation creation
handles.fpsPlay=100;
set(handles.popupmenu14,'Value',3); %FPS of PLAY button
handles.gaussSmooth=0.1;
set(handles.popupmenu15,'Value',1); %sigma of GAUSS SMOOTH button
%%% Initialize the sliders
set(handles.slider1,'sliderstep',[0 0],'Max',0,'Min',0,'Value',0)
set(handles.slider2,'sliderstep',[0 0],'Max',0,'Min',0,'Value',0)
handles.boost=0;



guidata(hObject,handles)


function text1_CreateFcn(hObject, eventdata, handles)

function text2_CreateFcn(hObject, eventdata, handles)

function text4_CreateFcn(hObject, eventdata, handles)

function text13_CreateFcn(hObject, eventdata, handles)

function axes1_CreateFcn(hObject, eventdata, handles)

function axes2_CreateFcn(hObject, eventdata, handles)

%% --- Executes on IMAGE slider movement.
function slider1_Callback(hObject, eventdata, handles)
frame=round(get(handles.slider1,'Value'));
ROI=round(get(handles.slider2,'Value'));

axes(handles.axes1)
hold off
if isfield(handles, 'raw_image')
    imagesc(handles.raw_image(:,:,frame))
    colormap(jet)
    caxis([mean(min(min(handles.raw_image))) mean(max(max(handles.raw_image)))])
    hold on
end

if isfield(handles,'ROI_position') && ROI~=0
    if ROI ~= handles.number_of_ROIs
        ROIboundary=handles.ROI_position{ROI};
        plot(ROIboundary(:,2),ROIboundary(:,1),'w','LineWidth',1);
    else
        for i=1:(handles.number_of_ROIs-1)
            ROIboundary=handles.ROI_position{i};
            plot(ROIboundary(:,2),ROIboundary(:,1),'w','LineWidth',1);
        end
    end
end

hold off
titl=sprintf('frame %g of %g',frame,handles.num_images);
title(titl)
axis square;
axis off;

if isfield(handles,'delF') && ROI~=0
    axes(handles.axes2) %initialize plot in axes2
    lim=ylim;
    binary=handles.binary;
    binary(binary==0)=nan;
    area(binary(:,ROI)*lim(2),'LineStyle','none','FaceColor',[0.8 0.8 0.8])
    hold on
    plot(handles.delF(:,ROI),'k')
    axis([1 handles.num_images min(handles.delF(:,ROI)) max(handles.delF(:,ROI))])
    ylabel('delF/Fo')
    xlabel('Image')
    ymax=get(gca,'ylim');
    
%     plot(find(handles.spikes(:,ROI)==1),handles.delF(find(handles.spikes(:,ROI)==1),ROI),'ro');
%     plot(find(handles.peak(:,ROI)==1),handles.delF(find(handles.peak(:,ROI)==1),ROI),'ko');
%     plot(find(handles.decayed(:,ROI)==1),handles.delF(find(handles.decayed(:,ROI)==1),ROI),'go');
    
    plot([frame frame],ymax)%vertical line of frame value
    hold off
end
guidata(hObject,handles)

function slider1_CreateFcn(hObject, eventdata, handles)

if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

%% --- Executes on GIF button press in pushbutton27.
function pushbutton27_Callback(hObject, eventdata, handles)

if isfield(handles, 'raw_image')
    filename=[handles.pathway handles.fname(1:length(handles.fname)-4) '.gif']
    current_frame=round(get(handles.slider1,'Value'));
    delay=1/100;
    frame=1;
    axes(handles.axes1)
    ROI=round(get(handles.slider2,'Value'));
    while frame<=handles.num_images
        imagesc(handles.raw_image(:,:,frame))
        colormap(jet)
        caxis([mean(min(min(handles.raw_image))) mean(max(max(handles.raw_image)))])
        hold on
        
        if isfield(handles,'ROI_position') && ROI~=0
            if ROI ~= handles.number_of_ROIs
                ROIboundary=handles.ROI_position{ROI};
                plot(ROIboundary(:,2),ROIboundary(:,1),'w','LineWidth',1);
            else
                for i=1:(handles.number_of_ROIs-1)
                    ROIboundary=handles.ROI_position{i};
                    plot(ROIboundary(:,2),ROIboundary(:,1),'w','LineWidth',1);
                end
            end
        end
        if frame >= 100 && frame <= 200
            plot(50,50,'r.','MarkerSize',72)
        end
        hold off
        titl=sprintf('frame %g of %g',frame,handles.num_images);
        title(titl)
        axis off;
        pause(0.005)
        gifframe = getframe(handles.axes1);    %Part of code to write GIF
        im = frame2im(gifframe);
        [imind,cm] = rgb2ind(im,256);
        if frame == 1;
            imwrite(imind,cm,filename,'gif','DelayTime',delay, 'Loopcount',inf);
        else
            imwrite(imind,cm,filename,'gif','DelayTime',delay, 'WriteMode','append');
        end
        frame=frame+1;
    end
    imagesc(handles.raw_image(:,:,current_frame))
    hold on
    if isfield(handles,'ROI_position') && ROI~=0
        if ROI ~= handles.number_of_ROIs
            ROIboundary=handles.ROI_position{ROI};
            plot(ROIboundary(:,2),ROIboundary(:,1),'w','LineWidth',1);
        else
            for i=1:(handles.number_of_ROIs-1)
                ROIboundary=handles.ROI_position{i};
                plot(ROIboundary(:,2),ROIboundary(:,1),'w','LineWidth',1);
                handles.ROI_label(i)=text(handles.CoM(i,1),handles.CoM(i,2),num2str(i));
                handles.ROI_label(i).Color=[1 1 1];
            end
        end
    end
    hold off
    titl=sprintf('frame %g of %g',current_frame,handles.num_images);
    title(titl)
    axis square;
    axis off;
    guidata(hObject,handles)
end


%% --- Executes on GIF FPS selection change in popupmenu11.
function popupmenu11_Callback(hObject, eventdata, handles)

contents = get(hObject, 'Value');
string=[1 2 3 4 5 6 7 8 9 10];
handles.fps=string(contents);
guidata(hObject,handles)

function popupmenu11_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


%% --- Executes on SNAP button press in pushbutton24.
function pushbutton24_Callback(hObject, eventdata, handles)

if isfield(handles, 'raw_image')
    axes(handles.axes1)
    frame = getframe;
    im = frame2im(frame);
    imwrite(im,[handles.pathway handles.fname(1:(length(handles.fname)-4)) '_snap.png'],'png');
    display([handles.pathway handles.fname(1:(length(handles.fname)-4)) '_snap.png SNAPPED'])
end

function popupmenu14_Callback(hObject, eventdata, handles)

contents = get(hObject, 'Value');
string=[1 10 100 1000];
handles.fpsPlay=string(contents);
guidata(hObject,handles)

function popupmenu14_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

%% --- Executes on PLAY button press.
function PLAY_Callback(hObject, eventdata, handles)

if isfield(handles, 'raw_image')
    current_frame=round(get(handles.slider1,'Value'));
    axes(handles.axes1)
    ROI=round(get(handles.slider2,'Value'));
    color=[mean(min(min(handles.raw_image))) mean(max(max(handles.raw_image)))];
    pause_time=1/handles.fpsPlay;
    imagesc(handles.raw_image(:,:,1))
    colormap(jet)
    caxis(color)
    frame=2;
    while frame<=handles.num_images
        imagesc(handles.raw_image(:,:,frame))
        caxis(color)
        hold on
        if isfield(handles,'ROI_position') && ROI~=0
            if ROI ~= handles.number_of_ROIs
                ROIboundary=handles.ROI_position{ROI};
                plot(ROIboundary(:,2),ROIboundary(:,1),'w','LineWidth',1);
            else
                for i=1:(handles.number_of_ROIs-1)
                    ROIboundary=handles.ROI_position{i};
                    plot(ROIboundary(:,2),ROIboundary(:,1),'w','LineWidth',1);
                end
            end
        end
        hold off
        titl=sprintf('frame %g of %g',frame,handles.num_images);
        title(titl)
        axis square;
        axis off;
        pause(pause_time)
        frame=frame+1;
    end
    imagesc(handles.raw_image(:,:,current_frame))
    hold on
    if isfield(handles,'ROI_position') && ROI~=0
        if ROI ~= handles.number_of_ROIs
            ROIboundary=handles.ROI_position{ROI};
            plot(ROIboundary(:,2),ROIboundary(:,1),'w','LineWidth',1);
        else
            for i=1:(handles.number_of_ROIs-1)
                ROIboundary=handles.ROI_position{i};
                plot(ROIboundary(:,2),ROIboundary(:,1),'w','LineWidth',1);
                handles.ROI_label(i)=text(handles.CoM(i,1),handles.CoM(i,2),num2str(i));
                handles.ROI_label(i).Color=[1 1 1];
            end
        end
    end
    hold off
    titl=sprintf('frame %g of %g',current_frame,handles.num_images);
    title(titl)
    axis square;
    axis off;
    guidata(hObject,handles)
end

%% --- Executes on UnBLEACH button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)

if isfield(handles,'raw_image')
    handles.raw_image=UnBleach(handles.raw_image,handles.num_images,handles.width,handles.height);
end
guidata(hObject,handles)

%% --- Executes on LOAD FUNCTIONAL TIFF button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)

if isfield(handles,'pathway')
    osearch='*.tif';
    search=[handles.pathway osearch];
else
    search='*.tif';
end
if isfield(handles,'raw_image')
    axes(handles.axes1)
    image(1);
    axis off
    axis square
end
if isfield(handles,'SR101image')
    handles=rmfield(handles,'SR101image');
end
if isfield(handles,'play_image')
    handles=rmfield(handles,'play_image');
end
if isfield(handles,'ROI_position')
    handles=rmfield(handles,'ROI_position');
    handles=rmfield(handles,'number_of_ROIs');
    handles=rmfield(handles,'ROI_masks');
    handles=rmfield(handles,'CoM');
    handles=rmfield(handles,'ROI_click');
    axes(handles.axes2)
    image(1);
    axis off
    set(handles.text2,'String','0')
    set(handles.text4,'String','0')
    set(handles.slider2,'sliderstep',[0 0],'Max',1,'Min',0,'Value',0)
end
if isfield(handles,'delF')
    handles=rmfield(handles,'delF');
    handles=rmfield(handles,'spikes');
end
if isfield(handles,'order')
    handles=rmfield(handles,'order');
end
if isfield(handles,'masq')
    handles=rmfield(handles,'masq');
end

[handles.fname handles.pathway]=uigetfile(search);

if handles.fname==0
    return
end

[handles.raw_image, handles.num_images, handles.height, handles.width]=LoadTiff(handles.fname, handles.pathway);
display(['Functional Image Loaded: ' handles.pathway handles.fname])

axes(handles.axes1) %change plot in axes1
imagesc(handles.raw_image(:,:,1))   %load the first image from the loaded stack to axes1
colormap(jet)
caxis([mean(min(min(handles.raw_image))) mean(max(max(handles.raw_image)))])
set(handles.slider1,'sliderstep',[1 10]/(handles.num_images-1),'Max',handles.num_images,'Min',1,'Value',1)  %setup slider to navigate image stack
axis off;
axis square;
titl=sprintf('frame %g of %g',1,handles.num_images);
title(titl)

matname=handles.fname;
matname(size(matname,2)-2:size(matname,2))='mat';
handles.matname = matname;     %set matname = filename for saving purposes at end of analysis
set(handles.text1,'String',matname(1:(length(matname)-4)))  %set title of program to image name

guidata(hObject,handles)    %saves all current handles. variables to guidata that can be used in subsequent functions

%% --- Executes on LOAD SR101 button press in pushbutton28.
function pushbutton28_Callback(hObject, eventdata, handles)

if isfield(handles,'pathway')
    osearch='*.tif';
    search=[handles.pathway osearch];
else
    search='*.tif';
end

[handles.fname handles.pathway]=uigetfile(search);

if handles.fname==0
    return
end

[handles.SR101image, ~, handles.height, handles.width]=LoadTiff(handles.fname, handles.pathway);
display(['Structural Image Loaded: ' handles.pathway handles.fname])

axes(handles.axes1)
image=sum(handles.SR101image,3);
handles.play_image=image;
red=[1:1:20]';
red=red./20;
map=[red zeros(size(red)) zeros(size(red))];
imagesc(image)
colormap(map)
brighten(0.3)
axis off
axis square
guidata(hObject,handles)


%% --- Executes on VIEW SR101 button press.
function SR101_Callback(hObject, eventdata, handles)

if isfield(handles,'num_images')
    if isfield(handles,'SR101image')
        axes(handles.axes1)
        image=sum(handles.SR101image,3);
        handles.play_image=image;
        red=[1:1:20]';
        red=red./20;
        map=[red zeros(size(red)) zeros(size(red))];
        imagesc(image)
        colormap(map)
        brighten(0.3)
        axis off
        axis square
        if isfield(handles,'CoM')
            ROI=round(get(handles.slider2,'Value'));
            hold on
            if ROI ~= handles.number_of_ROIs
                ROIboundary=handles.ROI_position{ROI};
                plot(ROIboundary(:,2),ROIboundary(:,1),'w','LineWidth',1);
            else
                for i=1:(handles.number_of_ROIs-1)
                    ROIboundary=handles.ROI_position{i};
                    plot(ROIboundary(:,2),ROIboundary(:,1),'w','LineWidth',1);
                end
            end
            hold off
        end
    else
        handles.SR101image=handles.raw_image(:,:,round(handles.num_images/2):handles.num_images);
        handles.raw_image=handles.raw_image(:,:,1:round(handles.num_images/2));
        handles.num_images=round(handles.num_images/2);
        
        axes(handles.axes1) %change plot in axes1
        
        imagesc(handles.raw_image(:,:,1))
        axis off;
        axis square;
        set(handles.slider1,'sliderstep',[1 10]/(handles.num_images-1),'Max',handles.num_images,'Min',1,'Value',1)  %setup slider to navigate image stack
        titl=sprintf('frame %g of %g',1,handles.num_images);
        title(titl)
        display('Structural Image Separated')
    end
    guidata(hObject,handles)
end

%% --- Executes on MEDIAN FILTER button press in pushbutton25.
function pushbutton25_Callback(hObject, eventdata, handles)

if isfield(handles,'raw_image')
    handles.raw_image=TemporalFilter(handles.raw_image, handles.num_images, handles.temporalFilter);
    display(['Temporal Medial Filter: ' num2str(handles.temporalFilter)])
    
    set(handles.slider1,'sliderstep',[1 10]/(handles.num_images-1),'Max',handles.num_images,'Min',1,'Value',1) %reset slider
    guidata(hObject,handles)
    axes(handles.axes1)
    title(['Temporal Medial Filter: ' num2str(handles.temporalFilter) ' done'])
end

%% --- Executes on MEDIAN FILTER PARAMETER selection change in popupmenu8.
function popupmenu8_Callback(hObject, eventdata, handles)

contents = get(hObject, 'Value');
string=[1 3 5 7 9 11];
handles.temporalFilter=string(contents);
guidata(hObject,handles)

function popupmenu8_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


%% --- Executes on PRUNE button press in pushbutton29.
function pushbutton29_Callback(hObject, eventdata, handles)

if isfield(handles,'number_of_ROIs')
    start=handles.number_of_ROIs;
    wait=waitbar(0,'Pruning the crop');
    ROI=bwareaopen(handles.ROI_masks(:,:,handles.number_of_ROIs),handles.clusterspace);
    [ROI_position,ROI_map,number_of_ROIs]=bwboundaries(ROI);
    ROI_position=ROI_position(1:number_of_ROIs);
    ROI_masks=zeros(handles.height,handles.width,number_of_ROIs);
    for i=1:number_of_ROIs
        ROI_masks(:,:,i)=ROI_map==i;
    end
    waitbar(1/3,wait)
    if number_of_ROIs==0    %If there are no ROIs found then prompt user and return
        CoM=[0 0];
        disp('All ROIs pruned');
        close(wait)
        return
    end
    ROI_masks(:,:,number_of_ROIs+1)=ROI;    %Make last ROI a combination of all ROIs
    CoM=zeros(number_of_ROIs,2);    %Find CoM of all ROIs used to label with ROInumber
    for ROI=1:number_of_ROIs
        ROIboundary=ROI_position{ROI};
        CoM(ROI,:)=[mean(ROIboundary(:,2)) mean(ROIboundary(:,1))];
    end
    handles.number_of_ROIs=number_of_ROIs+1;
    handles.ROI_masks=ROI_masks;
    handles.CoM=CoM;
    handles.ROI_click=nan(size(CoM));
    handles.ROI_position=ROI_position;
    waitbar(2/3,wait)
    set(handles.slider2,'sliderstep',[1 10]/(handles.number_of_ROIs-1),'Max',handles.number_of_ROIs,'Min',1,'Value',handles.number_of_ROIs)
    hold on
    for i=1:(handles.number_of_ROIs-1)
        ROIboundary=handles.ROI_position{i};
        plot(ROIboundary(:,2),ROIboundary(:,1),'w','LineWidth',1);
        handles.ROI_label(i)=text(handles.CoM(i,1),handles.CoM(i,2),num2str(i));
        handles.ROI_label(i).Color=[1 1 1];
    end
    hold off
    set(handles.text2,'String',handles.number_of_ROIs)
    set(handles.text4,'String',handles.number_of_ROIs)
    waitbar(1,wait)
    close(wait)
    guidata(hObject,handles)
    sprintf('ROIs smaller than %g have been pruned down to %g ROIs',handles.clusterspace, handles.number_of_ROIs);
end


%% --- Executes on FIND ROIS button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)

if isfield(handles,'raw_image')
    frame=round(get(handles.slider1,'Value'));
    if isfield(handles,'number_of_ROIs')
        handles=rmfield(handles,'ROI_position');
        handles=rmfield(handles,'number_of_ROIs');
        handles=rmfield(handles,'ROI_masks');
        handles=rmfield(handles,'CoM');
        handles=rmfield(handles,'ROI_click');
        axes(handles.axes2)
        image(1)
        axis off;
        set(handles.slider2,'sliderstep',[0 0],'Max',1,'Min',0,'Value',0)
        disp('All ROIs have been removed');
        set(handles.text4,'String','0')
        set(handles.text2,'String','0')
        axes(handles.axes1)
        imagesc(handles.raw_image(:,:,frame))
        axis off;
        axis square;
    end
    if isfield(handles,'delF')  %remove ROI's info from spike analysis if it exists
        handles=rmfield(handles,'delF');
        handles=rmfield(handles,'spikes');
        handles=rmfield(handles,'peak');
        handles=rmfield(handles,'decayed');
        handles=rmfield(handles,'binary');
    end
    display('---------------')
    fprintf('Auto find ROIs:\nBaseline Frames = %g\nStDev Threshold = %g\nSpatial Filter = %g\n',str2num(get(handles.edit2,'String')), str2num(get(handles.edit3,'String')), handles.spatialFilter)
    [handles.CoM, handles.ROI_position, handles.ROI_masks, handles.number_of_ROIs]=findROIs(handles.raw_image, handles.height, handles.width, handles.num_images, str2num(get(handles.edit2,'String')), str2num(get(handles.edit3,'String')), handles.spatialFilter);
    fprintf('Calsee found %g ROIs\n',handles.number_of_ROIs)
    handles.ROI_click=nan(handles.number_of_ROIs-1,2);
    if handles.number_of_ROIs>0
        set(handles.slider2,'sliderstep',[1 10]/(handles.number_of_ROIs-1),'Max',handles.number_of_ROIs,'Min',1,'Value',handles.number_of_ROIs)
        hold on
        for i=1:(handles.number_of_ROIs-1)
            ROIboundary=handles.ROI_position{i};
            plot(ROIboundary(:,2),ROIboundary(:,1),'w','LineWidth',1);
        end
        hold off
        set(handles.text2,'String',handles.number_of_ROIs)
        set(handles.text4,'String',handles.number_of_ROIs)
    end
end
guidata(hObject,handles)
display('---------------')


function popupmenu1_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

%% --- Executes on CLUSTER SIZE selection change in popupmenu2.
function popupmenu2_Callback(hObject, eventdata, handles)

contents = get(hObject, 'Value');
string=[8 16 32 128 256 512 1024 2048 4096];
handles.clusterspace=string(contents);
guidata(hObject,handles)

function popupmenu2_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit2_Callback(hObject, eventdata, handles)
%% BASELINE call back edit2

function edit2_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function edit3_Callback(hObject, eventdata, handles)
%% StDev Threshold call back edit3

function edit3_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


%% --- Executes on SPATIAL FILTER selection change in popupmenu12.
function popupmenu12_Callback(hObject, eventdata, handles)

contents = get(hObject, 'Value');
string=[1 3 5 7 9 11];
handles.spatialFilter=string(contents);
guidata(hObject,handles)


function popupmenu12_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function popupmenu13_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


%% --- Executes on ROI slider movement.
function slider2_Callback(hObject, eventdata, handles)
if isfield(handles, 'number_of_ROIs')
    if isfield(handles,'ROI_label')
        delete(handles.ROI_label)
    end
    ROI=round(get(handles.slider2,'Value'));
    frame=round(get(handles.slider1,'Value'));
    
    axes(handles.axes1)     %allows to play with figure
    hold on     %allow figure to plot image and ROI outline
    
    oldROI=findobj('type','line');
    delete(oldROI)     %delete old ROI outline from plot
    
    if ROI ~= handles.number_of_ROIs
        ROIboundary=handles.ROI_position{ROI};
        plot(ROIboundary(:,2),ROIboundary(:,1),'w','LineWidth',1);
    else
        for i=1:(handles.number_of_ROIs-1)
            ROIboundary=handles.ROI_position{i};
            plot(ROIboundary(:,2),ROIboundary(:,1),'w','LineWidth',1);
            handles.ROI_label(i)=text(handles.CoM(i,1),handles.CoM(i,2),num2str(i));
            handles.ROI_label(i).Color=[1 1 1];
        end
    end
    
    hold off    %allows the old ROI outlines to be deleted and so that the figure isn't a stack of images
    
    axis off;
    axis square;
    set(handles.text2,'String',ROI)
    
    if isfield(handles,'delF') && ROI~=0
        frame=round(get(handles.slider1,'Value'));
        axes(handles.axes2) %initialize plot in axes2
        plot(handles.delF(:,ROI),'k')
        axis([1 handles.num_images min(handles.delF(:,ROI)) max(handles.delF(:,ROI))])
        
        ymax=get(gca,'ylim');
        
        lim=ylim;
        binary=handles.binary;
        binary(binary==0)=nan;
        area(binary(:,ROI)*lim(2),'LineStyle','none','FaceColor',[0.8 0.8 0.8])
        hold on
        plot(handles.delF(:,ROI),'k')
        axis([1 handles.num_images min(handles.delF(:,ROI)) max(handles.delF(:,ROI))])
        ylabel('delF/Fo')
        xlabel('Image')
        
        
%         plot(find(handles.spikes(:,ROI)==1),handles.delF(find(handles.spikes(:,ROI)==1),ROI),'ro');
%         plot(find(handles.peak(:,ROI)==1),handles.delF(find(handles.peak(:,ROI)==1),ROI),'ko');
%         plot(find(handles.decayed(:,ROI)==1),handles.delF(find(handles.decayed(:,ROI)==1),ROI),'go');
        plot([frame frame],ymax)%vertical line of frame value
        hold off
    end
    
    guidata(hObject,handles)
end


function slider2_CreateFcn(hObject, eventdata, handles)

if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


%% --- Executes on DELETE ROI button press in pushbutton7.
function pushbutton7_Callback(hObject, eventdata, handles)

if isfield(handles,'ROI_position') && handles.number_of_ROIs>2
    frame=round(get(handles.slider1,'Value'));  %what frame do the sliders encode currently
    ROI=round(get(handles.slider2,'Value'));
    if ROI==handles.number_of_ROIs %delete all the ROIs
        handles=rmfield(handles,'ROI_position');
        handles=rmfield(handles,'number_of_ROIs');
        handles=rmfield(handles,'ROI_masks');
        handles=rmfield(handles,'CoM');
        handles=rmfield(handles,'ROI_click');
        axes(handles.axes2)
        image(1)
        axis off;
        set(handles.slider2,'sliderstep',[0 0],'Max',1,'Min',0,'Value',0)
        disp('All ROIs have been removed');
        set(handles.text4,'String','0')
        set(handles.text2,'String','0')
        axes(handles.axes1)
        imagesc(handles.raw_image(:,:,frame))
        axis off
        axis square
        guidata(hObject,handles)
        return
    end
    oldROI=findobj('type','line');  %find variable for outline overlay of current ROI
    delete(oldROI)                  %delete ROI outline from the figure
    handles.ROI_position(ROI)=[];   %remove this ROIs position from handles
    handles.ROI_masks(:,:,ROI)=[];
    handles.CoM(ROI,:)=[];
    handles.number_of_ROIs=handles.number_of_ROIs-1;    %1 less ROI
    handles.ROI_click(ROI,:)=[];
    if isfield(handles,'delF')  %remove ROI's info from spike analysis if it exists
        handles.delF(:,ROI)=[];
        handles.spikes(:,ROI)=[];
        handles.peak(:,ROI)=[];
        handles.decayed(:,ROI)=[];
    end
    set(handles.slider2,'sliderstep',[1 10]/(handles.number_of_ROIs-1),'Max',handles.number_of_ROIs,'Min',1,'Value',ROI)    %changed value from 1 -> ROI
    axes(handles.axes1)
    hold on
    if ROI ~= handles.number_of_ROIs
        ROIboundary=handles.ROI_position{ROI};
        plot(ROIboundary(:,2),ROIboundary(:,1),'w','LineWidth',1);
    else
        for i=1:(handles.number_of_ROIs-1)
            ROIboundary=handles.ROI_position{i};
            plot(ROIboundary(:,2),ROIboundary(:,1),'w','LineWidth',1);
            handles.ROI_label(i)=text(handles.CoM(i,1),handles.CoM(i,2),num2str(i));
            handles.ROI_label(i).Color=[1 1 1];
        end
    end
    hold off
    d=sprintf('ROI # %g has been discarded',ROI);
    disp(d)
    set(handles.text4,'String',handles.number_of_ROIs)  %update ROI text boxes
    set(handles.text2,'String',ROI)
    handles.ROI_masks(:,:,handles.number_of_ROIs)=zeros(handles.height,handles.width);  %recreate allROI mask
    for i=1:(handles.number_of_ROIs-1)
        handles.ROI_masks(:,:,handles.number_of_ROIs)=handles.ROI_masks(:,:,handles.number_of_ROIs)+handles.ROI_masks(:,:,i);
    end
    if isfield(handles,'delF') && ROI~=0
        axes(handles.axes2) %initialize plot in axes2
        plot(handles.delF(:,ROI)) %relative fluorescence
        axis([1 handles.num_images min(handles.delF(:,ROI)) max(handles.delF(:,ROI))])
        ylabel('Intensity')
        xlabel('Image')
        ymax=get(gca,'ylim');
        hold on
        plot(find(handles.spikes(:,ROI)==1),handles.delF(find(handles.spikes(:,ROI)==1),ROI),'ro');
        plot(find(handles.peak(:,ROI)==1),handles.delF(find(handles.peak(:,ROI)==1),ROI),'ko');
        plot(find(handles.decayed(:,ROI)==1),handles.delF(find(handles.decayed(:,ROI)==1),ROI),'go');
        plot([frame frame],ymax)%vertical line of frame value
        hold off
    end
elseif isfield(handles,'ROI_position') && handles.number_of_ROIs==2
    handles=rmfield(handles,'ROI_position');
    handles=rmfield(handles,'number_of_ROIs');
    handles=rmfield(handles,'ROI_masks');
    handles=rmfield(handles,'CoM');
    handles=rmfield(handles,'ROI_click');
    axes(handles.axes2)
    image(1)
    axis off;
    set(handles.slider2,'sliderstep',[0 0],'Max',1,'Min',0,'Value',0)
    disp('All ROIs have been removed');
    set(handles.text4,'String','0')
    set(handles.text2,'String','0')
    if isfield(handles,'delF');
        handles=rmfield(handles,'delF');
        handles=rmfield(handles,'spikes');
    end
else
    return
end
guidata(hObject,handles)

%% --- Executes on DRAW ROI button press in pushbutton8.
function pushbutton8_Callback(hObject, eventdata, handles)

if isfield(handles, 'raw_image')
    if isfield(handles,'ROI_position')
        oldROI=findobj('type','line');  %find variable for outline overlay of current ROI
        delete(oldROI)                  %remove that overlay on image
        frame=round(get(handles.slider1,'Value'));
        axes(handles.axes1)
        if isfield(handles, 'play_image')
            imagesc(handles.play_image) %preferentially use play image
        else
            imagesc(handles.raw_image(:,:,frame))
            titl=sprintf('frame %g of %g',frame,handles.num_images);
            title(titl)
        end
        axis off
        axis square;
        freehand=imfreehand(handles.axes1);      %free draw the ROI
        new_ROI_mask=freehand.createMask();     %make a mask of it
        handles.ROI_masks(:,:,handles.number_of_ROIs)=new_ROI_mask; %make new ROI the last ROI
        handles.number_of_ROIs=handles.number_of_ROIs+1; %increase the number of ROIs by 1
        handles.ROI_masks(:,:,handles.number_of_ROIs)=zeros(handles.height,handles.width); %recreate population ROI mask
        for i=1:(handles.number_of_ROIs-1)
            handles.ROI_masks(:,:,handles.number_of_ROIs)=handles.ROI_masks(:,:,handles.number_of_ROIs)+handles.ROI_masks(:,:,i);
        end
        new_ROI_position=bwboundaries(new_ROI_mask);
        handles.ROI_position{handles.number_of_ROIs-1}=new_ROI_position{1};
        ROI_position=new_ROI_position{1};
        handles.CoM(handles.number_of_ROIs-1,:)=[mean(ROI_position(:,2)) mean(ROI_position(:,1))];
        handles.ROI_click(handles.number_of_ROIs-1,:)=nan(1,2);
        hold on
        ROIboundary=handles.ROI_position{handles.number_of_ROIs-1};
        plot(ROIboundary(:,2),ROIboundary(:,1),'w','LineWidth',1);
        handles.ROI_label=text(handles.CoM(handles.number_of_ROIs-1,1),handles.CoM(handles.number_of_ROIs-1,2),num2str(handles.number_of_ROIs-1));
        handles.ROI_label.Color=[1 1 1];
        set(handles.slider2,'sliderstep',[1 10]/(handles.number_of_ROIs-1),'Max',handles.number_of_ROIs,'Min',1,'Value',handles.number_of_ROIs-1) %update ROI slider
        set(handles.text4,'String',handles.number_of_ROIs)  %update ROI text boxes
        set(handles.text2,'String',handles.number_of_ROIs-1)
        hold off
    else
        freehand=imfreehand(handles.axes1);     %draw by freehand a new ROI
        new_ROI_mask=freehand.createMask();     %create mask from freehand
        frame=round(get(handles.slider1,'Value'));
        axes(handles.axes1)
        if isfield(handles, 'play_image')
            imagesc(handles.play_image)
        else
            imagesc(handles.raw_image(:,:,frame))
        end
        axis off
        axis square
        new_ROI_position=bwboundaries(new_ROI_mask);
        handles.ROI_position=new_ROI_position;
        ROI_position=new_ROI_position{1};
        handles.CoM(1,:)=[mean(ROI_position(:,2)) mean(ROI_position(:,1))];
        handles.ROI_click(1,:)=nan(1,2);
        handles.ROI_masks(:,:,1)=new_ROI_mask; %make new ROI the last ROI
        handles.number_of_ROIs=2; %one for the ROI one for the population
        handles.ROI_masks(:,:,handles.number_of_ROIs)=new_ROI_mask; %recreate population ROI mask
        set(handles.slider2,'sliderstep',[1 10]/(handles.number_of_ROIs-1),'Max',handles.number_of_ROIs,'Min',1,'Value',handles.number_of_ROIs-1) %update ROI slider
        set(handles.text4,'String',handles.number_of_ROIs)  %update ROI text boxes
        set(handles.text2,'String',1)
        hold on
        ROIboundary=new_ROI_position{1};
        plot(ROIboundary(:,2),ROIboundary(:,1),'w','LineWidth',1);
        handles.ROI_label=text(handles.CoM(1,1),handles.CoM(1,2),num2str(1));
        handles.ROI_label.Color=[1 1 1];
        
        hold off
    end
    guidata(hObject,handles)
end

function popupmenu10_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

%% --- Executes on SAVE ROIS in pushbutton11.
function pushbutton11_Callback(hObject, eventdata, handles)

if isfield(handles,'ROI_position')
    dummy.ROI_position=handles.ROI_position;
    dummy.ROI_masks=handles.ROI_masks;
    dummy.number_of_ROIs=handles.number_of_ROIs;
    dummy.CoM=handles.CoM;
    dummy.ROI_click=handles.ROI_click;
    search=[handles.pathway '*.mat'];
    [ROIfile,pathway]=uiputfile(search);
    save_name=[pathway ROIfile];
    save([pathway ROIfile],'dummy')
end

%% --- Executes on LOAD ROIS in pushbutton12.
function pushbutton12_Callback(hObject, eventdata, handles)

if isfield(handles, 'raw_image')
    search=handles.pathway;
    [ROIfile,pathway]=uigetfile(search);
    load([pathway ROIfile])
    handles.ROI_position=dummy.ROI_position;
    handles.ROI_masks=dummy.ROI_masks;
    handles.number_of_ROIs=dummy.number_of_ROIs;
    handles.CoM=dummy.CoM;
    if isfield(dummy,'ROI_click')
        handles.ROI_click=dummy.ROI_click;
    else
        handles.ROI_click=nan(handles.number_of_ROIs-1,2);
    end
    if handles.number_of_ROIs>0
        set(handles.slider2,'sliderstep',[1 10]/(handles.number_of_ROIs-1),'Max',handles.number_of_ROIs,'Min',1,'Value',handles.number_of_ROIs)
        hold on
        for i=1:(handles.number_of_ROIs-1)  %Draw on the ROI outlines
            ROIboundary=handles.ROI_position{i};
            plot(ROIboundary(:,2),ROIboundary(:,1),'w','LineWidth',1);
            handles.ROI_label(i)=text(handles.CoM(i,1),handles.CoM(i,2),num2str(i));
            handles.ROI_label(i).Color=[1 1 1];
        end
        hold off
        set(handles.text2,'String',handles.number_of_ROIs)
        set(handles.text4,'String',handles.number_of_ROIs)
    end
    guidata(hObject,handles)
end


%% --- Executes on ANALYSIS button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)

if isfield(handles,'ROI_masks')
    [handles.spikes,handles.delF,handles.peak,handles.decayed,handles.binary]=CaSpike(handles.number_of_ROIs, handles.num_images, handles.ROI_masks, handles.raw_image, str2num(get(handles.edit4,'String')), str2num(get(handles.edit5,'String')));
    frame=round(get(handles.slider1,'Value'));
    ROI=round(get(handles.slider2,'Value'));
    axes(handles.axes2) %initialize plot in axes2
    
    plot(handles.delF(:,ROI)) %maximum value
    axis([1 handles.num_images min(handles.delF(:,ROI)) max(handles.delF(:,ROI))])
    ylabel('delF/Fo')
    xlabel('Image')
    ymax=get(gca,'ylim');
    hold on
    
    lim=ylim;
    binary=handles.binary;
    binary(binary==0)=nan;
    area(binary(:,ROI)*lim(2),'LineStyle','none','FaceColor',[0.8 0.8 0.8])
    
    plot(handles.delF(:,ROI),'k') %maximum value
    
%     plot(find(handles.spikes(:,ROI)==1),handles.delF(find(handles.spikes(:,ROI)==1),ROI),'ro');
%     plot(find(handles.peak(:,ROI)==1),handles.delF(find(handles.peak(:,ROI)==1),ROI),'ko');
%     plot(find(handles.decayed(:,ROI)==1),handles.delF(find(handles.decayed(:,ROI)==1),ROI),'go');
    plot([frame frame],ymax)%vertical line of frame value
    hold off
    guidata(hObject,handles)
    display('Analysis Complete')
end

%% --- Executes on EXPORT TO XLSX in pushbutton13.
function pushbutton13_Callback(hObject, eventdata, handles)

if isfield(handles,'delF')
    CoM=[handles.CoM(:,1) handles.CoM(:,2)]';
    population_CoM=[mean(handles.CoM(:,1)); mean(handles.CoM(:,2))];
    coordinates=[CoM population_CoM];
    x=[1:1:handles.number_of_ROIs];
    excel=[x;coordinates;handles.delF];
    search=[handles.pathway '*.xlsx'];
    [XLSfile, pathway]=uiputfile(search);
    if XLSfile(length(XLSfile)-3:length(XLSfile))=='xlsx'
        XLSfile=XLSfile(1:length(XLSfile-4));
    end
    if exist([pathway XLSfile],'file')
        delete([pathway XLSfile]);
        display(['Overwriting ' XLSfile])
    end
    xlswrite([pathway XLSfile],excel)
end

%% --- Executes on VISUALIZE button press in pushbutton14.
function pushbutton14_Callback(hObject, eventdata, handles)

if isfield(handles,'delF') && handles.number_of_ROIs>2
    traces=handles.delF;
%     traces=(traces-repmat(mean(traces(1:handles.baseline,:)),size(traces,1),1))./repmat(mean(traces(1:handles.baseline,:)),size(traces,1),1);
    traces(:,end)=[]; %Remove population trace
    
    figure
    surf(traces','facecolor','interp','edgecolor','none')
    colormap(jet)
    title('Traces')
    xlabel('Frame')
    ylabel('ROIs')
    zlabel('delF/F')
    
    figure
    imagesc(traces')
    colormap(jet)
    caxis([mean(min(min(traces))) mean(max(max(traces)))])
    set(gca,'YTick',[1:1:handles.number_of_ROIs-1])
    title('Heat Map')
    xlabel('Frame')
    ylabel('ROIs')
    
    figure
    [roi ind]=find(handles.spikes');
    scatter(ind,roi)
%     imagesc(handles.spikes');
    set(gca,'YTick',[1:handles.number_of_ROIs-1])
    title('Raster')
    xlabel('Frame')
    ylabel('ROIs')
else
    display('More ROIs needed to visualize')
end

%% --- Executes on SAVE RASTER button press in pushbutton15.
function pushbutton15_Callback(hObject, eventdata, handles)

if isfield(handles,'spikes')
    CoM=[handles.CoM(:,1) handles.CoM(:,2)]';
    population_CoM=[mean(handles.CoM(:,1)); mean(handles.CoM(:,2))];
    coordinates=[CoM population_CoM];
    x=[1:1:handles.number_of_ROIs];
    excel=[x;coordinates;handles.spikes];   %First sheet
    
    [roi,ind]=find(handles.spikes);         %Second sheet
    excel2=[ind roi];
    
    search=[handles.pathway '*.xlsx'];
    [XLSfile, pathway]=uiputfile(search);
    if XLSfile(length(XLSfile)-3:length(XLSfile))=='xlsx'
        XLSfile=XLSfile(1:length(XLSfile-4));
    end
    if exist([pathway XLSfile],'file')
        delete([pathway XLSfile]);
        display(['Overwriting old file' XLSfile])
    end
    xlswrite([pathway XLSfile],excel)
    xlswrite([pathway XLSfile],excel2,2)
    axes(handles.axes1)
    title(['Saved Raster'])
end

%% --- Executes on SAVE BINARY button press in pushbutton33.
function pushbutton33_Callback(hObject, eventdata, handles)

if isfield(handles,'binary')
    x=[1:1:handles.number_of_ROIs];
    excel=[x;handles.binary];

    search=[handles.pathway '*.xlsx'];
    [XLSfile, pathway]=uiputfile(search);
    if XLSfile(length(XLSfile)-3:length(XLSfile))=='xlsx'
        XLSfile=XLSfile(1:length(XLSfile-4));
    end
    if exist([pathway XLSfile],'file')
        delete([pathway XLSfile]);
        display(['Overwriting old file' XLSfile])
    end
    xlswrite([pathway XLSfile],excel)
    axes(handles.axes1)
    title(['Saved Binary'])
end


%%  Load Tiff Image
function [raw_image, num_images, height, width]=LoadTiff(filename,pathway)

fname=[pathway filename];
imginfo=imfinfo(fname);
num_images=numel(imginfo);
height=imginfo(1).Height;
width=imginfo(1).Width;
raw_image=uint16(zeros(height,width,num_images));
wait=waitbar(0,'Reading in Image');
for i=1:num_images      %read in image
    raw_image(:,:,i)=imread(fname,'Index',i);
    waitbar(i/num_images,wait)
end
close(wait)
fclose('all');


%% UnBleach
function [unbleached]=UnBleach(image,num_images,width,height)

if isfield(handles, 'raw_image')
    image=double(image);
    meaner=mean(mean(image,1),2);
    meaner=meaner(:);
    %Find fit
    x=[1:1:length(meaner)]';
    f = @(b,x) b(1).*exp(-b(2).*x) + b(3);                  %Function
    OLS = @(b) sum((f(b,x) - meaner).^2);
    opts = optimset('MaxFunEvals',5000, 'MaxIter',5000);    %Fitting parameters
    B = fminsearch(OLS, [18 0 min(meaner)],opts);           %Fit with initial values
    fcnfit = f(B,x);                                        %Behold, le fit
    unbleacheder=(min(fcnfit))+(meaner-fcnfit);
    unbleached=zeros(size(image));
    wait=waitbar(0.3,'uNbLeAcHiNg');
    for i=1:width
        waitbar((i/width)*(0.7)+0.3,wait)
        for j=1:height
            trace=image(i,j,:);
            trace=trace(:);
            unbleached(i,j,:)=trace-unbleacheder;
        end
    end
    close(wait)
end

%% Temporal Low-pass Filter
function [smoothed]=TemporalFilter(raw_image,num_images,temporalSmooth)
smoothed=medfilt3(raw_image,[1 1 temporalSmooth]);
smoothed(:,:,1:(temporalSmooth-1)/2)=repmat(smoothed(:,:,((temporalSmooth-1)/2)+1),1,1,(temporalSmooth-1)/2); %fix edge effects from smoothing
smoothed(:,:,end-(temporalSmooth-1)/2:end)=repmat(smoothed(:,:,(end-(temporalSmooth-1)/2)-1),1,1,(temporalSmooth-1)/2+1);

%%  Automatically locate ROIs
function [CoM,ROI_position,ROI_masks,number_of_ROIs]=findROIs(raw_image,height,width,num_images,winlength,stdThreshold,spatialfilter)

%  Create initial ROIs
wait=waitbar(0,'grabbing ROIs from activity');
image=double(raw_image);
clear raw_image

% Old code reliant on a static threshold
% thresh=mean(image(:,:,1:winlength),3)+stdThreshold*std(image(:,:,1:winlength),[],3);
% thresh=repmat(thresh,1,1,num_images);

% ROI detection based on moving threshold
meaner=movmean(image,[winlength 0],3);
stder=movstd(image,[winlength 0],0,3);
stder(:,:,1:2*winlength)=repmat(mean(stder(:,:,2*winlength),3),1,1,2*winlength);  %Trying to remove blip in early trace
thresh=meaner+stdThreshold*stder;
prethresh=repmat(mean(thresh(:,:,winlength:2*winlength),3),1,1,2*winlength);
postthresh=thresh(:,:,1:end-2*winlength);
thresh(:,:,1:2*winlength)=prethresh;
thresh(:,:,2*winlength+1:end)=postthresh;

intensity_mask=image>thresh;
clear image thresh

for r=1:size(intensity_mask,3)  %I did develop a 3d median filter, but it wasn't as good as this.
    intensity_mask(:,:,r)=medfilt2(intensity_mask(:,:,r),[spatialfilter spatialfilter]);
end
ROI=medfilt2(sum(intensity_mask,3)>1,[spatialfilter spatialfilter])>0;

waitbar(1/3,wait)

%  Isolate ROIs and create matrix of ROI positions
[ROI_position,ROI_map,number_of_ROIs]=bwboundaries(ROI);
ROI_position=ROI_position(1:number_of_ROIs);
ROI_masks=zeros(height,width,number_of_ROIs);
for i=1:number_of_ROIs
    ROI_masks(:,:,i)=ROI_map==i;
end
waitbar(2/3,wait)
if number_of_ROIs==0    %If there are no ROIs found then prompt user and return
    CoM=[0 0];
    disp('No ROIs found');
    close(wait)
    return
end
ROI_masks(:,:,number_of_ROIs+1)=ROI;    %Make last ROI a combination of all ROIs
CoM=zeros(number_of_ROIs,2);    %Find CoM of all ROIs used to label with ROInumber
for ROI=1:number_of_ROIs
    ROIboundary=ROI_position{ROI};
    CoM(ROI,:)=[mean(ROIboundary(:,2)) mean(ROIboundary(:,1))];
end
waitbar(1,wait)
number_of_ROIs=number_of_ROIs+1;
close(wait)


%%  Spike counting analysis
function [raster,delF,peak,decay,binary] =CaSpike(num_rois,num_images,ROI_masks,raw_image,std_thresh,winlength)

wait=waitbar(0,'Analyzing ROIs');
delF=zeros(num_images,num_rois);
raster=zeros(num_images,num_rois);
peak=raster;
decay=raster;
binary=raster;
raw_image=double(raw_image);

%%%% PARAMETERS %%%%
bump=4; %how many entries over the threshold
%Using 4 enforces the age-old rule of at least 3 points for an event.
%%%%%%%%%%%%%%%%%%%%

for i=1:num_rois
    waitbar(i/num_rois,wait)
    ROI_image=repmat(ROI_masks(:,:,i),1,1,num_images).*raw_image;
    ROI_image(ROI_image==0)=nan;
    trace=nanmean(nanmean(ROI_image,1),2);
    delF(:,i)=(trace);    %normalize delF/Fo
    
    positive_count=0;
    track=0;
    amp=[];
    wide=[];
    rise=[];
    back=[];
    for r=1:length(delF(:,i))
        if r<=winlength
            threshold(r)=mean(delF(1:winlength,i))+std_thresh*std(delF(1:winlength,i));
        else
            threshold(r)=mean(delF(r-winlength:r-1,i))+std_thresh*std(delF(r-winlength:r-1,i));
        end
        if positive_count>0
            %Threshold is not updated
        else
            thresh=threshold(r-positive_count);
        end
        thru(r,i)=thresh;   %Remember moving threshold
        if delF(r,i)>thresh   %This is to remove false positives (noise) that are only up once
            positive_count=positive_count+1;
        else
            positive_count=0;
        end
        if positive_count>=bump && track==0
            track=1;
            start=r-bump+1;
            raster(start,i)=1;
            startamp=delF(start,i);
            tripped=threshold(start);
        end
        if track==1 && (delF(r,i)<tripped) || track==1 && r==length(delF(:,i))
            track=0;
            decay(r,i)=1;
            [maximus,ind]=max(delF(start:r,i));
            peak(start+ind-1,i)=1;
            positive_count=0;
            amp(end+1)=maximus-delF(start,i);
            wide(end+1)=r-start;
            rise(end+1)=ind;
            back(end+1)=r-start-ind;
            binary(start:r,i)=ones(r-start+1,1);
        end
    end
    amplitude(i)=mean(amp);
    width(i)=mean(wide);
    risetime(i)=mean(rise);
    decaytime(i)=mean(back);
end
close(wait)

% --- Executes when figure1 is resized.
function figure1_SizeChangedFcn(hObject, eventdata, handles)


%% --- Executes on MEAN t-STACK button press in pushbutton16.
function pushbutton16_Callback(hObject, eventdata, handles)

if isfield(handles, 'raw_image')
    handles.play_image=mean(handles.raw_image,3);
    axes(handles.axes1)
    imagesc(handles.play_image)
    colormap(jet)
    axis off;
    axis square;
    title('Mean Image')
    handles.meaner=mean(handles.play_image(:));
    handles.stder=std(handles.play_image(:));
    ROI=round(get(handles.slider2,'Value'));
    if isfield(handles,'ROI_position')
        hold on
        if ROI ~= handles.number_of_ROIs
            ROIboundary=handles.ROI_position{ROI};
            plot(ROIboundary(:,2),ROIboundary(:,1),'w','LineWidth',1);
        else
            for i=1:(handles.number_of_ROIs-1)
                ROIboundary=handles.ROI_position{i};
                plot(ROIboundary(:,2),ROIboundary(:,1),'w','LineWidth',1);
                %             handles.ROI_label(i)=text(handles.CoM(i,1),handles.CoM(i,2),num2str(i));
                %             handles.ROI_label(i).Color=[1 1 1];
            end
        end
        hold off
    end
    guidata(hObject,handles)
end

%% --- Executes on MAXIMUM t-STACK button press in pushbutton17.
function pushbutton17_Callback(hObject, eventdata, handles)

if isfield(handles, 'raw_image')
    handles.play_image=max(handles.raw_image,[],3);
    axes(handles.axes1)
    imagesc(handles.play_image)
    colormap(jet)
    axis off;
    axis square;
    title('Maximum Image')
    handles.play_image=double(handles.play_image);
    handles.meaner=mean(handles.play_image(:));
    handles.stder=std(handles.play_image(:));
    ROI=round(get(handles.slider2,'Value'));
    if isfield(handles,'ROI_position')
        hold on
        if ROI ~= handles.number_of_ROIs
            ROIboundary=handles.ROI_position{ROI};
            plot(ROIboundary(:,2),ROIboundary(:,1),'w','LineWidth',1);
        else
            for i=1:(handles.number_of_ROIs-1)
                ROIboundary=handles.ROI_position{i};
                plot(ROIboundary(:,2),ROIboundary(:,1),'w','LineWidth',1);
                handles.ROI_label(i)=text(handles.CoM(i,1),handles.CoM(i,2),num2str(i));
                handles.ROI_label(i).Color=[1 1 1];
            end
        end
        hold off
    end
    guidata(hObject,handles)
end

%% --- Executes on ST DEV t-STACK button press in pushbutton18.
function pushbutton18_Callback(hObject, eventdata, handles)

if isfield(handles, 'raw_image')
    image=double(handles.raw_image);
    std_image=std(image,0,3);
    axes(handles.axes1)
    imagesc(std_image)
    colormap(jet)
    axis off;
    axis square;
    title('Standard Deviation Image')
    handles.play_image=std_image;
    handles.meaner=mean(mean(handles.play_image));
    handles.stder=std(std(handles.play_image));
    ROI=round(get(handles.slider2,'Value'));
    if isfield(handles,'ROI_position')
        hold on
        if ROI ~= handles.number_of_ROIs
            ROIboundary=handles.ROI_position{ROI};
            plot(ROIboundary(:,2),ROIboundary(:,1),'w','LineWidth',1);
        else
            for i=1:(handles.number_of_ROIs-1)
                ROIboundary=handles.ROI_position{i};
                plot(ROIboundary(:,2),ROIboundary(:,1),'w','LineWidth',1);
                handles.ROI_label(i)=text(handles.CoM(i,1),handles.CoM(i,2),num2str(i));
                handles.ROI_label(i).Color=[1 1 1];
            end
        end
        hold off
    end
    guidata(hObject,handles)
end


%% --- Executes on SR101 MEAN button press in pushbutton22.
function pushbutton22_Callback(hObject, eventdata, handles)

if isfield(handles, 'SR101image')
    handles.play_image=mean(handles.SR101image,3);
    axes(handles.axes1)
    imagesc(handles.play_image)
    colormap(jet)
    axis off;
    axis square;
    title('Mean Image')
    handles.meaner=mean(mean(handles.play_image));
    handles.stder=std(std(handles.play_image));
    ROI=round(get(handles.slider2,'Value'));
    if isfield(handles,'ROI_position')
        hold on
        if ROI ~= handles.number_of_ROIs
            ROIboundary=handles.ROI_position{ROI};
            plot(ROIboundary(:,2),ROIboundary(:,1),'w','LineWidth',1);
        else
            for i=1:(handles.number_of_ROIs-1)
                ROIboundary=handles.ROI_position{i};
                plot(ROIboundary(:,2),ROIboundary(:,1),'w','LineWidth',1);
                %             handles.ROI_label(i)=text(handles.CoM(i,1),handles.CoM(i,2),num2str(i));
                %             handles.ROI_label(i).Color=[1 1 1];
            end
        end
        hold off
    end
    guidata(hObject,handles)
end


%% --- Executes on CLICK CELL button press in pushbutton21.
function pushbutton21_Callback(hObject, eventdata, handles)

if isfield(handles, 'number_of_ROIs')
    ROI=round(get(handles.slider2,'Value'));
    if isfield(handles,'ROI_position') && ROI~=0
        if ROI==handles.number_of_ROIs
            axes(handles.axes1)
            title('** Cannot segment with the population ROI **')
        else
            ROI_mask=handles.ROI_masks(:,:,ROI);
            image=double(handles.raw_image);
            if isfield(handles,'play_image')
                sd=handles.play_image;
            else
                sd=std(image,0,3);  %compute standard deviation image
            end
            axes(handles.axes1) %plot sd image on screen for user
            imagesc(sd)
            colormap(jet)
            axis off
            axis square
            hold on
            if isfield(handles,'ROI_position') && ROI~=0
                ROIboundary=handles.ROI_position{ROI};
                plot(ROIboundary(:,2),ROIboundary(:,1),'w','LineWidth',1);
            end
            hold off
            if isnan(handles.ROI_click(ROI,:))
                title('\fontsize{16}\color{red}Click the center of the soma')
                g=ginput(1);    %user select the cell soma
                handles.ROI_click(ROI,:)=g;
            else
                title('Working from previous click')
                g=handles.ROI_click(ROI,:);
            end
            cell=zeros(handles.height,handles.width);   %initialize masks
            process=cell;
            measurements=regionprops(ROI_mask,'Area','EquivDiameter');
            diameter=measurements.EquivDiameter;
            limit=2*diameter; %should be greater than radius of ROI
            limits=[g(1) g(2) handles.height-g(1) handles.width-g(2)];
            border=min(limits); %This should be the closest side of the image if there is one
            if border < limit   %restrict area of search if along edge of image
                limit=floor(border)-1;
            end
            for r=2:floor(limit)
                mask=zeros(size(sd));
                for th=1:(2*round(2*pi*r))
                    theta=2*pi*(th/(2*round(2*pi*r)));
                    x=round(r*cos(theta) +g(1));
                    y=round(r*sin(theta) +g(2));
                    mask(y,x)=1;
                    ring(th)=sd(y,x);
                end
%                 xlswrite(['ring' num2str(r) '.xlsx'],ring')
                meaner(r)=mean(ring);
                medianer(r)=median(ring);
                stder(r)=std(ring);
                cell=cell+(mask.*sd);
                for th=1:(round(2*pi*r))
                    theta=2*pi*(th/(round(2*pi*r)));
                    x=round(r*cos(theta) +g(1));
                    y=round(r*sin(theta) +g(2));
                    if cell(y,x) >= (medianer(r)+handles.process_threshold*stder(r))   %default was 1, then I changed it to 0.75, but should depend on area of ROI aka cell clicked
                        process(y,x)=1; %determine processes mask
                    end
                end
            end
            meaner(1)=meaner(2);
            floor(limit)
%             xlswrite('meaner.xlsx',meaner')
            rad=find(meaner<(((max(meaner)-min(meaner))*handles.soma_threshold)+min(meaner)));
            radius=rad(1);  %determine radius of soma
            [a,b]=meshgrid(-(g(1)-1):(handles.width-g(1)),-(g(2)-1):(handles.height-g(2))); %create a circular mask of soma estimate
            soma=((a.^2+b.^2)<=radius^2);
            filt_cell=medfilt2(cell).*ROI_mask;
            xmin=round(g(1)-limit);
            xmax=round(g(1)+limit);
            ymin=round(g(2)-limit);
            ymax=round(g(2)+limit);
            if xmin==0
                xmin=1;
            end
            if xmax==0
                xmax=1;
            end
            if ymin==0
                ymin=1;
            end
            if ymax==0
                ymax=1;
            end
            cropped_cell=filt_cell(ymin:ymax,xmin:xmax);
            filt_soma=soma;
            filt_process=medfilt2(process);
            handles.cell=cropped_cell;
            handles.soma=filt_soma(ymin:ymax,xmin:xmax);
            
%             figure
%             imagesc(handles.cell);
%             red=gray;
%             red(:,2:3)=0;
%             colormap(red)
%             axis square
%             axis off
            
%             figure
%             imagesc(handles.soma);
% %             imagesc(handles.soma.*handles.cell);
%             red=gray;
%             red(:,2:3)=0;
% %             colormap(red)
%             colormap(gray)
%             axis square
%             axis off
            
            filt_process=filt_process.*ROI_mask;
            handles.process=filt_process(ymin:ymax,xmin:xmax);
            
%             andmask=handles.process.*handles.soma;
%             justprocessmask=handles.process-andmask;
%             
% %             andmask=(handles.cell>0).*handles.soma;
% %             justprocessmask=(handles.cell>0)-andmask;
%             figure
% %             imagesc(justprocessmask.*handles.cell);
%             imagesc(justprocessmask);
%             red=gray;
%             red(:,2:3)=0;
% %             colormap(red)
%             colormap(gray)
%             axis square
%             axis off
            
            handles.cropped=handles.raw_image(ymin:ymax,xmin:xmax,:);
            handles.roi=ROI;
            handles.baseline=str2num(get(handles.edit2,'String'));
%             title('Click Cell window enabled')
            handles.winlength=str2num(get(handles.edit5,'String'));
            handles.threshold=str2num(get(handles.edit4,'String'));
            [clicker,handles.soma,handles.process]=ClickCell(handles); %Call ClickCell GUI
            title('')
            if clicker==0   %Cancel
                handles.ROI_click(ROI,:)=nan(1,2);
            elseif clicker==1   %Accept button press
                handles.ROI_click(ROI,:)=g;
                %ADDED to grab clickcell in roi coordinate
                tip=round(g-size(handles.soma')/2); %used to place subcellular ROIs on coordinate of cell ROI
                soma_mask=zeros(handles.height,handles.width);
                proc_mask=soma_mask;
                proc_mask(tip(2):tip(2)+size(handles.soma,1)-1,tip(1):tip(1)+size(handles.soma,2)-1)=handles.process;
                soma_mask(tip(2):tip(2)+size(handles.soma,1)-1,tip(1):tip(1)+size(handles.soma,2)-1)=handles.soma*2;
                masq=proc_mask+soma_mask;
                masq(find(masq==3))=2;
                if isfield(handles,'masq')
                    handles.masq(:,:,end+1)=masq;
                else
                    handles.masq=masq;
                end
            elseif clicker==2   %Reclick
                handles.ROI_click(ROI,:)=nan(1,2);
                clicker=[];
                handles = rmfield(handles,'process');
                handles = rmfield(handles,'soma');
                handles = rmfield(handles,'cell');
                handles = rmfield(handles,'cropped');
                handles = rmfield(handles,'roi');
                pushbutton21_Callback(hObject, eventdata, handles)
            end
            handles = rmfield(handles,'process');
            handles = rmfield(handles,'soma');
            handles = rmfield(handles,'cell');
            handles = rmfield(handles,'cropped');
            handles = rmfield(handles,'roi');
            guidata(hObject,handles)
        end
    end
end

%% --- Executes on SOMA THRESHOLD selection change in popupmenu5.
function popupmenu5_Callback(hObject, eventdata, handles)

contents = get(hObject, 'Value');
string=[0 0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1];
handles.soma_threshold=string(contents);
guidata(hObject,handles)

% --- Executes during object creation, after setting all properties.
function popupmenu5_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


%% --- Executes on PROCESS THRESHOLD selection change in popupmenu6.
function popupmenu6_Callback(hObject, eventdata, handles)

contents = get(hObject, 'Value');
string=[0 0.05 0.1 0.15 0.2 0.25 0.3 0.35 0.4];
handles.process_threshold=string(contents);
guidata(hObject,handles)

% --- Executes during object creation, after setting all properties.
function popupmenu6_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end




%% --- Executes on BASH button press in pushbutton30.
function pushbutton30_Callback(hObject, eventdata, handles)

if isfield(handles,'ROI_position') && isfield(handles,'play_image') && length(handles.ROI_click)==(handles.number_of_ROIs-1)
    % ANALYSIS
    [handles.spikes,handles.delF,handles.peak,handles.decayed,handles.binary]=CaSpike(handles.number_of_ROIs, handles.num_images, handles.ROI_masks, handles.raw_image, str2num(get(handles.edit4,'String')), str2num(get(handles.edit5,'String')));

    frame=round(get(handles.slider1,'Value'));
    ROI=round(get(handles.slider2,'Value'));
    axes(handles.axes2) %initialize plot in axes2
    plot(handles.delF(:,ROI),'k')
    lim=ylim;
    binary=handles.binary;
    binary(binary==0)=nan;
    area(binary(:,ROI)*lim(2),'LineStyle','none','FaceColor',[0.8 0.8 0.8])
    hold on
    plot(handles.delF(:,ROI),'k')
    axis([1 handles.num_images min(handles.delF(:,ROI)) max(handles.delF(:,ROI))])
    ylabel('delF/Fo')
    xlabel('Image')
    ymax=get(gca,'ylim');
    plot([frame frame],ymax)%vertical line of frame value
    hold off
    guidata(hObject,handles)
    
    
    
    %EXCEL
    CoM=[handles.CoM(:,1) handles.CoM(:,2)]';
    population_CoM=[mean(handles.CoM(:,1)); mean(handles.CoM(:,2))];
    coordinates=[CoM population_CoM];
    x=[1:1:handles.number_of_ROIs];
    excel=[x;coordinates;handles.delF];
    pathway=handles.pathway;
    XLSfile=[handles.fname(1:2) '.xlsx'];
    if XLSfile(length(XLSfile)-3:length(XLSfile))=='xlsx'
        XLSfile=XLSfile(1:length(XLSfile-4));
    end
    if exist([pathway XLSfile],'file')
        delete([pathway XLSfile]);
        display(['Overwriting ' XLSfile])
    end
    xlswrite([pathway XLSfile],excel)
    
    %CLICKCELL
    handles.boost=1;
    guidata(hObject,handles)
    for roi=1:handles.number_of_ROIs-1
        set(handles.slider2,'Value',roi)
        pushbutton21_Callback(hObject, eventdata, handles)  %clickcell button
    end
    handles.boost=0;
    guidata(hObject,handles)
    display('BASH Complete')
    axes(handles.axes1)
    title('BASH Complete')
    load gong
    sound(y,Fs)
end


%% --- Executes on RE-SOMA button press in pushbutton31.
function pushbutton31_Callback(hObject, eventdata, handles)

if isfield(handles,'ROI_click')
    handles.ROI_click=nan(size(handles.ROI_click));
    guidata(hObject,handles)
end


%% --- Executes on GAUSSIAN BLUR button press in pushbutton32.
function pushbutton32_Callback(hObject, eventdata, handles)
if isfield(handles,'raw_image')
    handles.raw_image=imgaussfilt3(handles.raw_image,[handles.gaussSmooth handles.gaussSmooth 0.01]);
    guidata(hObject,handles)
    display(['Gaussian Blur: ' num2str(handles.gaussSmooth)])
    axes(handles.axes1)
    title(['Gaussian Blur: ' num2str(handles.gaussSmooth) ' done'])
end

%% --- Executes on GAUSSIAN BLUR selection change in popupmenu15.
function popupmenu15_Callback(hObject, eventdata, handles)

contents = get(hObject, 'Value');
string=[0.1 0.3 0.5 0.7 0.9];
handles.gaussSmooth=string(contents);
guidata(hObject,handles)

% --- Executes during object creation, after setting all properties.
function popupmenu15_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit4_Callback(hObject, eventdata, handles)
%% StDev Trace Threshold call back edit4

function edit4_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit5_Callback(hObject, eventdata, handles)
%% Moving Window Length call back edit5

function edit5_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end




%% --- Executes on DEBUGGING button press in pushbutton6.
function pushbutton6_Callback(hObject, eventdata, handles)

handles

size(handles.masq)
masq=sum(handles.masq,3);

figure
imagesc(medfilt2(masq))
axis off
axis square
colormap([1 1 1;1 0 0;0 0 1])
caxis([0 2])
