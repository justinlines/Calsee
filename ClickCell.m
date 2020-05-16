function varargout = ClickCell(varargin)
% CLICKCELL MATLAB code for ClickCell.fig
%      CLICKCELL, by itself, creates a new CLICKCELL or raises the existing
%      singleton*.
%
%      H = CLICKCELL returns the handle to a new CLICKCELL or the handle to
%      the existing singleton*.
%
%      CLICKCELL('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in CLICKCELL.M with the given input arguments.
%
%      CLICKCELL('Property','Value',...) creates a new CLICKCELL or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before ClickCell_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to ClickCell_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help ClickCell

% Last Modified by GUIDE v2.5 20-Jan-2020 14:35:57

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @ClickCell_OpeningFcn, ...
                   'gui_OutputFcn',  @ClickCell_OutputFcn, ...
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


% --- Executes just before ClickCell is made visible.
function ClickCell_OpeningFcn(hObject, eventdata, handles, varargin)

handles.output=0;
handles.cell = varargin{1}.cell;
handles.soma = varargin{1}.soma;
handles.process = varargin{1}.process;
handles.image=im2double(varargin{1}.cropped);
[handles.height,handles.width]=size(handles.cell);
handles.matname=varargin{1}.matname;
handles.roi=varargin{1}.roi;
handles.num_images=size(handles.image,3);
handles.pathway=varargin{1}.pathway;
handles.winlength=varargin{1}.winlength;
handles.threshold=varargin{1}.threshold;
handles.boost=varargin{1}.boost;
% Update handles structure
guidata(hObject, handles);

    %% INITIALIZE %%
% Get default command line output from handles structure

axes(handles.axes2), image(1); axis off
axes(handles.axes4), axis off
axes(handles.axes1)
imagesc(handles.cell)

% colormap(gray)
red=gray;
red(:,2:3)=0;
colormap(red)
axis square
axis off

set(handles.text2,'String',['ROI ' num2str(handles.roi) ' in file ' num2str(handles.matname(1:(length(handles.matname)-4)))])

stamp=ones(handles.height,handles.width)-handles.soma;  %remove soma from processes mask
handles.process=handles.process.*stamp;

handles.process=bwareaopen(handles.process,20); %make ROIs of each process segment
[position,map,num_ROIs]=bwboundaries(handles.process,'nohole');
mapplus=map+ones(handles.height,handles.width);
map=mapplus.*handles.process;
map=map+handles.soma;
num_ROIs=num_ROIs+1;    %because the soma
cell_mask=map>0;

ROI_mask=zeros(handles.height,handles.width,num_ROIs);  %put em together
for i=1:num_ROIs
    ROI_mask(:,:,i)=map==i;
end

ROI_mask(:,:,num_ROIs+1)=handles.process;
ROI_mask(:,:,num_ROIs+2)=cell_mask;

soma_position=bwboundaries(handles.soma);
position=[soma_position;position];  %add soma position to ROI positions

CoM=zeros(num_ROIs,2);    %Find CoM of all ROIs used to label with ROInumber
for ROI=1:num_ROIs
    ROIboundary=position{ROI};
    CoM(ROI,:)=[mean(ROIboundary(:,2)) mean(ROIboundary(:,1))];
end

num_ROIs=num_ROIs+2;    %Account for process combination and process with soma combination

set(handles.text3,'String',num2str(num_ROIs))   %show population ROI
set(handles.text5,'String',num2str(num_ROIs))


handles.ROI_position=position;

wait=waitbar(0,'Analyzing subcellular domains');
for roi=1:num_ROIs
    waitbar(roi/num_ROIs,wait)
    ROI_image=zeros(handles.height,handles.width);
    for r=1:handles.num_images
        ROI_image(:,:)=ROI_mask(:,:,roi).*handles.image(:,:,r);
        ROI_image(ROI_image==0)=nan;
        average(r,roi)=nanmean(nanmean(ROI_image,1),2);
    end
    average(:,roi)=(average(:,roi));
end
close(wait)

hold on
for i=1:num_ROIs-2
    ROIboundary=handles.ROI_position{i};
    plot(ROIboundary(:,2),ROIboundary(:,1),'w','LineWidth',1);
end

axes(handles.axes2)
plot(average(:,num_ROIs)), xlabel('Time (frame)'), ylabel('delF/Fo')
handles.number_of_ROIs=num_ROIs;
set(handles.slider1,'sliderstep',[1 10]/(handles.number_of_ROIs-1),'Max',handles.number_of_ROIs,'Min',1,'Value',num_ROIs)

handles.CoM=CoM;
handles.average=average;
handles.ROI_mask=ROI_mask;


wait=waitbar(0,'Analyzing ROIs');
spikes=zeros(handles.num_images,handles.number_of_ROIs);
peak=spikes;
decayed=spikes;
binary=spikes;

for roi=1:handles.number_of_ROIs
    waitbar(roi/handles.number_of_ROIs,wait)
    bump=4;
    start=0;
    winlength=handles.winlength;
    positive_count=0;
    track=0;
    
    std_thresh=handles.threshold;
    
    for r=1:handles.num_images          %Ca Event based off Fluorescence - CODE
        if r<=winlength
            threshold(r)=mean(handles.average(1:winlength,roi))+std_thresh*std(handles.average(1:winlength,roi));
        else
            threshold(r)=mean(handles.average(r-winlength:r-1,roi))+std_thresh*std(handles.average(r-winlength:r-1,roi));
        end
        if positive_count>0
            %Threshold is not updated
        else
            thresh=threshold(r-positive_count);
        end
        thru(r)=thresh;
        if handles.average(r,roi)>thresh
            positive_count=positive_count+1;
        else
            positive_count=0;
        end
        if positive_count>=bump && track==0
            track=1;
            spikes(r-bump+1,roi)=1;
            start=r-bump+1;
            tripped=threshold(start);
        end
        if track==1 && (handles.average(r,roi)<tripped) || track==1 && r==length(handles.average(:,roi))
            track=0;
            decayed(r,roi)=1;
            [amplitude, peak_location]=max(handles.average(start:r,roi));
            peak(peak_location+start-1,roi)=1;
            binary(start:r,roi)=ones(r-start+1,1);
        end
    end
end
handles.spikes=spikes;
handles.peak=peak;
handles.decayed=decayed;
handles.binary=binary;

close(wait)
ROI=round(get(handles.slider1,'Value'));
axes(handles.axes2)
lim=ylim;
hold off
binary(binary==0)=nan;
area(binary(:,ROI)*lim(2),'LineStyle','none','FaceColor',[0.8 0.8 0.8])
hold on
plot(average(:,num_ROIs)), xlabel('Time (frame)'), ylabel('Fluorescence (AU)')
set(handles.slider1,'sliderstep',[1 10]/(handles.number_of_ROIs-1),'Max',handles.number_of_ROIs,'Min',1,'Value',num_ROIs)
hold off
handles.boop=0;
guidata(hObject,handles)

if handles.boost==1
    pushbutton10_Callback(hObject, eventdata, handles)
else
    uiwait(handles.figure1);    % UIWAIT makes ClickCell wait for user response (see UIRESUME)
end



%% --- Outputs from this function are returned to the command line.
function varargout = ClickCell_OutputFcn(hObject, eventdata, handles) 

varargout{1} = handles.output;
varargout{2} = handles.soma;
varargout{3} = handles.process;
% The figure can be deleted now
delete(handles.figure1);


%% In case you want to check if this window is closed
% --- Executes when user attempts to close the figure.
function figure1_CloseRequestFcn(hObject, eventdata, handles)

if isequal(get(hObject, 'waitstatus'), 'waiting')
    % The GUI is still in UIWAIT, us UIRESUME
    uiresume(hObject);
elseif handles.boost==1
    if handles.boop==1
        delete(hObject);
    else
        handles.boop=1;
    end
else
    % The GUI is no longer waiting, just close it
    delete(hObject);
end


%% --- Executes on ACCEPT button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)

handles.output=1;
guidata(hObject,handles)
close


%% --- Executes on CANCEL button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)

handles.output=0;
guidata(hObject,handles)
close


%% --- Executes on ROI slider movement.
function slider1_Callback(hObject, eventdata, handles)

ROI=round(get(handles.slider1,'Value'));
set(handles.text3,'String',num2str(ROI))
hold off
axes(handles.axes1)
imagesc(handles.cell)
% colormap(gray)
red=gray;
red(:,2:3)=0;
colormap(red)
axis square
axis off
hold on
if isfield(handles,'ROI_position') && ROI~=0
    if ROI ~= handles.number_of_ROIs && ROI ~= handles.number_of_ROIs-1
        ROIboundary=handles.ROI_position{ROI};
        label=get(handles.checkbox2, 'Value');
        plot(ROIboundary(:,2),ROIboundary(:,1),'w','LineWidth',1);
        if label==1;
            handles.ROI_label=text(handles.CoM(ROI,1),handles.CoM(ROI,2),num2str(ROI));
            handles.ROI_label.Color=[1 0 0];
        end
    elseif ROI == handles.number_of_ROIs-1
        for i=2:(handles.number_of_ROIs-2)
            ROIboundary=handles.ROI_position{i};
            plot(ROIboundary(:,2),ROIboundary(:,1),'w','LineWidth',1);
            label= get(handles.checkbox2, 'Value');
            if label==1
                handles.ROI_label(i)=text(handles.CoM(i,1),handles.CoM(i,2),num2str(i));
                handles.ROI_label(i).Color=[1 0 0];
            end
        end
    else
        for i=1:(handles.number_of_ROIs-2)
            ROIboundary=handles.ROI_position{i};
            plot(ROIboundary(:,2),ROIboundary(:,1),'w','LineWidth',1);
            label= get(handles.checkbox2, 'Value');
            if label==1
                handles.ROI_label(i)=text(handles.CoM(i,1),handles.CoM(i,2),num2str(i));
                handles.ROI_label(i).Color=[1 0 0];
            end
        end
    end
end

axes(handles.axes2)
plot(handles.average(:,ROI)), xlabel('Time (frame)'), ylabel('Fluorescence (AU)')
value = get(handles.checkbox1, 'Value');
if value == 1 && ROI~=1
    hold on
    plot(handles.average(:,1),'w')
end

lim=ylim;
hold off
binary=handles.binary;
binary(binary==0)=nan;
area(binary(:,ROI)*lim(2),'LineStyle','none','FaceColor',[0.8 0.8 0.8])
hold on
plot(handles.average(:,ROI)), xlabel('Time (frame)'), ylabel('Fluorescence (AU)')
value = get(handles.checkbox1, 'Value');
if value == 1 && ROI~=1
    plot(handles.average(:,1),'w')
end
hold off


% --- Executes during object creation, after setting all properties.
function slider1_CreateFcn(hObject, eventdata, handles)

if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


%% --- Executes on SAVE EXCEL button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)

if isfield(handles,'average')
    CoM=[handles.CoM(:,1) handles.CoM(:,2)]';
    population_CoM=[mean(handles.CoM(:,1)); mean(handles.CoM(:,2))];
    coordinates=[CoM population_CoM population_CoM];
    x=[1:1:handles.number_of_ROIs];
    excel=[x;coordinates;handles.average];
    search=[handles.pathway '*.xlsx'];
    [XLSfile, pathway]=uiputfile(search);
    if XLSfile(length(XLSfile)-3:length(XLSfile))=='xlsx'
        XLSfile=XLSfile(1:length(XLSfile-4));
    end
    if exist([handles.pathway XLSfile],'file')
        delete([handles.pathway XLSfile]);
        display(['over writing ' XLSfile])
    end
    xlswrite([pathway XLSfile],excel)
end


%% --- Executes on SAVE RASTER button press in pushbutton12.
function pushbutton12_Callback(hObject, eventdata, handles)

if isfield(handles,'spikes')
    CoM=[handles.CoM(:,1) handles.CoM(:,2)]';
    population_CoM=[mean(handles.CoM(:,1)); mean(handles.CoM(:,2))];
    coordinates=[CoM population_CoM population_CoM];
    x=[1:1:handles.number_of_ROIs];
    excel=[x;coordinates;handles.spikes];
    
    [roi,ind]=find(handles.spikes);         %Second sheet
    excel2=[ind roi];
    
    search=[handles.pathway '*.xlsx'];
    [XLSfile, pathway]=uiputfile(search);
    if XLSfile(length(XLSfile)-3:length(XLSfile))=='xlsx'
        XLSfile=XLSfile(1:length(XLSfile-4));
    end
    if exist([handles.pathway XLSfile],'file')
        delete([handles.pathway XLSfile]);
        display(['over writing ' XLSfile])
    end
    xlswrite([pathway XLSfile],excel)
    xlswrite([pathway XLSfile],excel2,2)
    axes(handles.axes1)
    title(['Saved Raster'])
end


%% --- Executes on SAVE BINARY button press in pushbutton13.
function pushbutton13_Callback(hObject, eventdata, handles)

if isfield(handles,'binary')
    CoM=[handles.CoM(:,1) handles.CoM(:,2)]';
    population_CoM=[mean(handles.CoM(:,1)); mean(handles.CoM(:,2))];
    coordinates=[CoM population_CoM population_CoM];
    x=[1:1:handles.number_of_ROIs];
    excel=[x;coordinates;handles.binary];
    search=[handles.pathway '*.xlsx'];
    [XLSfile, pathway]=uiputfile(search);
    if XLSfile(length(XLSfile)-3:length(XLSfile))=='xlsx'
        XLSfile=XLSfile(1:length(XLSfile-4));
    end
    if exist([handles.pathway XLSfile],'file')
        delete([handles.pathway XLSfile]);
        display(['over writing ' XLSfile])
    end
    xlswrite([pathway XLSfile],excel)
end


%% --- Executes on ADD SOMA TRACE button press in checkbox1.
function checkbox1_Callback(hObject, eventdata, handles)

ROI=round(get(handles.slider1,'Value'));
axes(handles.axes2)
plot(handles.average(:,ROI)), xlabel('Time (frame)'), ylabel('Fluorescence (AU)')
value = get(handles.checkbox1, 'Value');
if value == 1 && ROI~=1
    hold on
    plot(handles.average(:,1),'w')
end
lim=ylim;
hold off
binary=handles.binary;
binary(binary==0)=nan;
area(binary(:,ROI)*lim(2),'LineStyle','none','FaceColor',[0.8 0.8 0.8])
hold on
plot(handles.average(:,ROI)), xlabel('Time (frame)'), ylabel('delF/Fo')
value = get(handles.checkbox1, 'Value');
if value == 1 && ROI~=1
    plot(handles.average(:,1),'w')
end
hold off

%% --- Executes on DISCRETIZE CELL button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)

xmax=handles.width;
ymax=handles.height;
xmin=1;
ymin=1;
working_mask=handles.ROI_mask(:,:,handles.number_of_ROIs-1);
soma_mask=handles.ROI_mask(:,:,1);
soma_position=handles.ROI_position(1);
soma_CoM=handles.CoM(1,:);
soma_average=handles.average(:,1);
handles.ROI_mask(:,:,1:handles.number_of_ROIs-1)=[];   %remove any trace of the ROI to be discretized
handles=rmfield(handles,'ROI_position');
handles.CoM=[];
handles.average(:,1:handles.number_of_ROIs-1)=[];
num_ROIs=1;  %-3 becuase I removed the current ROI from the masks initially
ROI_mask(:,:,num_ROIs)=soma_mask;
handles.ROI_position(num_ROIs)=soma_position;
handles.CoM(num_ROIs,:)=soma_CoM;
handles.average(:,num_ROIs)=soma_average;

wait=waitbar(0,'Analyzing subcellular domains');
% block=3;
block=5;
for x=xmin:block:xmax-block
    waitbar(x/(xmax-block),wait)
    for y=ymin:block:ymax-block
        stamp=zeros(handles.height,handles.width);
        if nnz(working_mask(y:y+block-1,x:x+block-1))>(block^2)/3
            stamp(y:y+block-1,x:x+block-1)=ones(block,block);
            mask=stamp.*working_mask;
            num_ROIs=num_ROIs+1;
            ROI_mask(:,:,num_ROIs)=mask;
            position=bwboundaries(mask,'nohole');
            handles.ROI_position(num_ROIs)=position(1);
            ROIboundary=position{1};
            handles.CoM(num_ROIs,:)=[mean(ROIboundary(:,2)) mean(ROIboundary(:,1))];  %update CoM
            average=zeros(1,handles.num_images);
            for im=1:handles.num_images
                ROI_image=mask.*handles.image(:,:,im);
                ROI_image(ROI_image==0)=nan;
                average(im)=nanmean(nanmean(ROI_image,1),2);
            end
            handles.average(:,num_ROIs)=(average);
        end
    end
end
close(wait)
process_mask=zeros(handles.height,handles.width);   %recreate process and allcell mask
all_ROIs_mask=process_mask;
for i=1:num_ROIs
    if i==1
        all_ROIs_mask=all_ROIs_mask+ROI_mask(:,:,i);
    else
        all_ROIs_mask=all_ROIs_mask+ROI_mask(:,:,i);
        process_mask=process_mask+ROI_mask(:,:,i);
    end
end
ROI_mask(:,:,num_ROIs+1)=process_mask;
ROI_mask(:,:,num_ROIs+2)=all_ROIs_mask;

for i=1:2   %create traces of processes and cell masks
    average=zeros(1,handles.num_images);
    for im=1:handles.num_images
        ROI_image=ROI_mask(:,:,num_ROIs+i).*handles.image(:,:,im);
        ROI_image(ROI_image==0)=nan;
        average(im)=nanmean(nanmean(ROI_image,1),2);
    end
    handles.average(:,num_ROIs+i)=(average);
end

handles.number_of_ROIs=num_ROIs+2;
handles.ROI_mask=ROI_mask;
set(handles.slider1,'sliderstep',[1 10]/(handles.number_of_ROIs-1),'Max',handles.number_of_ROIs,'Min',1,'Value',handles.number_of_ROIs)
set(handles.text3,'String',num2str(handles.number_of_ROIs))
set(handles.text5,'String',num2str(handles.number_of_ROIs))

axes(handles.axes1)
imagesc(handles.cell)
red=gray;
red(:,2:3)=0;
colormap(red)
axis square
axis off

hold on
for i=1:handles.number_of_ROIs-2
    ROIboundary=handles.ROI_position{i};
    plot(ROIboundary(:,2),ROIboundary(:,1),'w','LineWidth',1);
    labels=get(handles.checkbox2, 'Value');
    if labels==1
        handles.ROI_label=text(handles.CoM(i,1),handles.CoM(i,2),num2str(i));
        handles.ROI_label.Color=[0 0 0];
    end
end
axes(handles.axes2)
plot(handles.average(:,handles.number_of_ROIs)), xlabel('Time (frame)'), ylabel('delF/Fo')

wait=waitbar(0,'Analyzing ROIs');
spikes=zeros(handles.num_images,handles.number_of_ROIs);
peak=spikes;
decayed=spikes;
binary=spikes;

for roi=1:handles.number_of_ROIs
    waitbar(roi/handles.number_of_ROIs,wait)
    bump=4;
    start=0;
    positive_count=0;
    track=0;
    
    std_thresh=handles.threshold;
    winlength=handles.winlength;
    
    for r=1:handles.num_images          %Ca Event based off Fluorescence - CODE
        if r<=winlength
            threshold(r)=mean(handles.average(1:winlength,roi))+std_thresh*std(handles.average(1:winlength,roi));
        else
            threshold(r)=mean(handles.average(r-winlength:r-1,roi))+std_thresh*std(handles.average(r-winlength:r-1,roi));
        end
        if positive_count>0
            %Threshold is not updated
        else
            thresh=threshold(r-positive_count);
        end
        thru(r)=thresh;
        if handles.average(r,roi)>thresh
            positive_count=positive_count+1;
        else
            positive_count=0;
        end
        if positive_count>=bump && track==0
            track=1;
            spikes(r-bump+1,roi)=1;
            start=r-bump+1;
            tripped=threshold(start);
        end
        if track==1 && (handles.average(r,roi)<tripped) || track==1 && r==length(handles.average(:,roi))
            track=0;
            decayed(r,roi)=1;
            [amplitude, peak_location]=max(handles.average(start:r,roi));
            peak(peak_location+start-1,roi)=1;
            binary(start:r,roi)=ones(r-start+1,1);
        end
    end
end
handles.spikes=spikes;
handles.peak=peak;
handles.decayed=decayed;
handles.binary=binary;

close(wait)
ROI=round(get(handles.slider1,'Value'));
axes(handles.axes2)
value = get(handles.checkbox1, 'Value');
if value == 1 && ROI~=1
    hold on
    plot(handles.average(:,1),'w')
end

lim=ylim;
hold off
binary=handles.binary;
binary(binary==0)=nan;
area(binary(:,ROI)*lim(2),'LineStyle','none','FaceColor',[0.8 0.8 0.8])
hold on
plot(handles.average(:,handles.number_of_ROIs)), xlabel('Time (frame)'), ylabel('delF/Fo')
value = get(handles.checkbox1, 'Value');
if value == 1 && ROI~=1
    plot(handles.average(:,1),'w')
end
hold off

handles.process=handles.ROI_mask(:,:,end-1)>0;
guidata(hObject,handles)

%% --- Executes on VISUALIZE button press in pushbutton9.
function pushbutton9_Callback(hObject, eventdata, handles)

if handles.number_of_ROIs > 2
    traces=handles.average;
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
end

%% --- Executes on ORDER ROIS button press in pushbutton6.
function pushbutton6_Callback(hObject, eventdata, handles)

if isfield(handles,'spikes')
    time_mask=zeros(handles.height,handles.width);
    for roi=1:handles.number_of_ROIs-2
        hit=find(handles.spikes(:,roi)==1);
        if roi==1 && isempty(hit)   %if the roi has no spikes
            soma=handles.winlength;  %set it to stim
        elseif roi==1
            soma=hit(1);
        end
        
        if isempty(hit) %if no spikes found
            hit(1)=soma;
        end
        
        hit(1)=hit(1)-soma;
        time_mask=time_mask+hit(1)*handles.ROI_mask(:,:,roi);
    end
    
    stamp=handles.ROI_mask(:,:,handles.number_of_ROIs);
    stamp(stamp==0)=NaN;
    time_mask=time_mask.*stamp;
    time_mask(time_mask==0)=NaN;
    
    axes(handles.axes1)
    hold off
    pcolor([time_mask nan(handles.height,1); nan(1,handles.width+1)]);
    shading flat;
    set(gca, 'ydir', 'reverse');
    caxis([-5 5])
    
    colormap(jet)
    axis equal
    axis off
    
    
    try 
        matlabImage = imread('ClickCellColorbar.png');
        axes(handles.axes4)
        image(matlabImage)
        axis off
        axis image
    catch me
    end
end


%% --- Executes on SHOW ROI LABELS button press in checkbox2.
function checkbox2_Callback(hObject, eventdata, handles)

ROI=round(get(handles.slider1,'Value'));
label=get(handles.checkbox2, 'Value');
axes(handles.axes1)

label=get(handles.checkbox2, 'Value');
if label==1;
    hold on
    if ROI ~= handles.number_of_ROIs && ROI ~= handles.number_of_ROIs-1
        handles.ROI_label=text(handles.CoM(ROI,1),handles.CoM(ROI,2),num2str(ROI));
        handles.ROI_label.Color=[1 0 0];
    elseif ROI == handles.number_of_ROIs-1
        for i=2:(handles.number_of_ROIs-2)
            handles.ROI_label(i)=text(handles.CoM(i,1),handles.CoM(i,2),num2str(i));
            handles.ROI_label(i).Color=[1 0 0];
        end
    else
        for i=1:(handles.number_of_ROIs-2)
            handles.ROI_label(i)=text(handles.CoM(i,1),handles.CoM(i,2),num2str(i));
            handles.ROI_label(i).Color=[1 0 0];
        end
    end
else
    hold off
    axes(handles.axes1)
    imagesc(handles.cell)
%     colormap(gray)
    red=gray;
    red(:,2:3)=0;
    colormap(red)
    axis square
    axis off
    hold on
    if isfield(handles,'ROI_position') && ROI~=0
        if ROI ~= handles.number_of_ROIs && ROI ~= handles.number_of_ROIs-1
            ROIboundary=handles.ROI_position{ROI};
            plot(ROIboundary(:,2),ROIboundary(:,1),'w','LineWidth',1);
        elseif ROI == handles.number_of_ROIs-1
            for i=2:(handles.number_of_ROIs-2)
                ROIboundary=handles.ROI_position{i};
                plot(ROIboundary(:,2),ROIboundary(:,1),'w','LineWidth',1);
            end
        else
            for i=1:(handles.number_of_ROIs-2)
                ROIboundary=handles.ROI_position{i};
                plot(ROIboundary(:,2),ROIboundary(:,1),'w','LineWidth',1);
            end
        end
    end
end


%% --- Executes on STREAM button press in pushbutton10.
function pushbutton10_Callback(hObject, eventdata, handles)

% First excel write
CoM=[handles.CoM(:,1) handles.CoM(:,2)]';
population_CoM=[mean(handles.CoM(:,1)); mean(handles.CoM(:,2))];
coordinates=[CoM population_CoM population_CoM];
x=[1:1:handles.number_of_ROIs];
excel=[x;coordinates;handles.average];

if length(num2str(handles.roi)) == 1
    ROI=['0' num2str(handles.roi)];
else
    ROI=num2str(handles.roi);
end

XLSfile=[num2str(handles.matname(1:(length(handles.matname)-4))) '-' ROI '-1.xlsx'];

if exist([handles.pathway XLSfile],'file')
    delete([handles.pathway XLSfile]);
    display(['over writing ' XLSfile])
end

xlswrite([handles.pathway XLSfile],excel)


% Discretize
xmax=handles.width;
ymax=handles.height;
xmin=1;
ymin=1;
working_mask=handles.ROI_mask(:,:,handles.number_of_ROIs-1);
soma_mask=handles.ROI_mask(:,:,1);
soma_position=handles.ROI_position(1);
soma_CoM=handles.CoM(1,:);
soma_average=handles.average(:,1);
handles.ROI_mask(:,:,1:handles.number_of_ROIs-1)=[];   %remove any trace of the ROI to be discretized
handles=rmfield(handles,'ROI_position');
handles.CoM=[];
handles.average(:,1:handles.number_of_ROIs-1)=[];
num_ROIs=1;  %-3 becuase I removed the current ROI from the masks initially
ROI_mask(:,:,num_ROIs)=soma_mask;
handles.ROI_position(num_ROIs)=soma_position;
handles.CoM(num_ROIs,:)=soma_CoM;
handles.average(:,num_ROIs)=soma_average;

wait=waitbar(0,'Analyzing subcellular domains');
block=3;
for x=xmin:block:xmax-block
    waitbar(x/(xmax-block),wait)
    for y=ymin:block:ymax-block
        stamp=zeros(handles.height,handles.width);
        if nnz(working_mask(y:y+block-1,x:x+block-1))>(block^2)/3
            stamp(y:y+block-1,x:x+block-1)=ones(block,block);
            mask=stamp.*working_mask;
            num_ROIs=num_ROIs+1;
            ROI_mask(:,:,num_ROIs)=mask;
            position=bwboundaries(mask,'nohole');
            handles.ROI_position(num_ROIs)=position(1);
            ROIboundary=position{1};
            handles.CoM(num_ROIs,:)=[mean(ROIboundary(:,2)) mean(ROIboundary(:,1))];  %update CoM
            average=zeros(1,handles.num_images);
            for im=1:handles.num_images
                ROI_image=mask.*handles.image(:,:,im);
                ROI_image(ROI_image==0)=nan;
                average(im)=nanmean(nanmean(ROI_image,1),2);
            end
            handles.average(:,num_ROIs)=(average);
        end
    end
end
close(wait)

process_mask=zeros(handles.height,handles.width);   %recreate process and allcell mask
all_ROIs_mask=process_mask;
for i=1:num_ROIs
    if i==1
        all_ROIs_mask=all_ROIs_mask+ROI_mask(:,:,i);
    else
        all_ROIs_mask=all_ROIs_mask+ROI_mask(:,:,i);
        process_mask=process_mask+ROI_mask(:,:,i);
    end
end
ROI_mask(:,:,num_ROIs+1)=process_mask;
ROI_mask(:,:,num_ROIs+2)=all_ROIs_mask;

for i=1:2   %create traces of processes and cell masks
    average=zeros(1,handles.num_images);
    for im=1:handles.num_images
        ROI_image=ROI_mask(:,:,num_ROIs+i).*handles.image(:,:,im);
        ROI_image(ROI_image==0)=nan;
        average(im)=nanmean(nanmean(ROI_image,1),2);
    end
    handles.average(:,num_ROIs+i)=(average);
end

handles.number_of_ROIs=num_ROIs+2;
handles.ROI_mask=ROI_mask;
set(handles.slider1,'sliderstep',[1 10]/(handles.number_of_ROIs-1),'Max',handles.number_of_ROIs,'Min',1,'Value',handles.number_of_ROIs)
set(handles.text3,'String',num2str(handles.number_of_ROIs))
set(handles.text5,'String',num2str(handles.number_of_ROIs))

axes(handles.axes1)
imagesc(handles.cell)
% colormap(gray)
red=gray;
red(:,2:3)=0;
colormap(red)
axis square
axis off

hold on
for i=1:handles.number_of_ROIs-2
    ROIboundary=handles.ROI_position{i};
    plot(ROIboundary(:,2),ROIboundary(:,1),'w','LineWidth',1);
    labels=get(handles.checkbox2, 'Value');
    if labels==1
        handles.ROI_label=text(handles.CoM(i,1),handles.CoM(i,2),num2str(i));
        handles.ROI_label.Color=[1 0 0];
    end
end
axes(handles.axes2)
plot(handles.average(:,handles.number_of_ROIs)), xlabel('Time (frame)'), ylabel('Fluorescence (AU)')
% 
% wait=waitbar(0,'Analyzing ROIs');
% spikes=zeros(handles.num_images,handles.number_of_ROIs);
% peak=spikes;
% decayed=spikes;
% 
% for roi=1:handles.number_of_ROIs
%     waitbar(roi/handles.number_of_ROIs,wait)
%     bump=4;
%     start=0;
%     winlength=handles.winlength;
%     positive_count=0;
%     track=0;
%     
%     std_thresh=handles.threshold;
%     
%     for r=1:handles.num_images          %Ca Event based off Fluorescence - CODE
%         if r<=winlength
%             threshold(r)=mean(handles.average(1:winlength,roi))+std_thresh*std(handles.average(1:winlength,roi));
%         else
%             threshold(r)=mean(handles.average(r-winlength:r,roi))+std_thresh*std(handles.average(r-winlength:r,roi));
%         end
%         if positive_count>0
%             %Then threshold is not updated
%         else
%             thresh=threshold(r-positive_count);
%         end
%         if handles.average(r,roi)>thresh
%             positive_count=positive_count+1;
%         else
%             positive_count=0;
%         end
%         if positive_count>=bump && track==0
%             track=1;
%             spikes(r-bump+1,roi)=1;
%             start=r-bump+1;
%             tripped=threshold(start);
%         end
%         if track==1 && (handles.average(r,roi)<tripped) || track==1 && r==length(handles.average(:,roi))
%             track=0;
%             decayed(r,roi)=1;
%             [amplitude, peak_location]=max(handles.average(start:r,roi));
%             peak(peak_location+start-1,roi)=1;
%             binary(start:r,roi)=ones(r-start+1,1);
%         end
%     end
% end
% handles.spikes=spikes;
% handles.peak=peak;
% handles.decayed=decayed;
% handles.binary=binary;
% 
% close(wait)
% ROI=round(get(handles.slider1,'Value'));
% axes(handles.axes2)
% value = get(handles.checkbox1, 'Value');
% if value == 1 && ROI~=1
%     hold on
%     plot(handles.average(:,1),'r')
% end
% 
% lim=ylim;
% hold off
% binary=handles.binary;
% binary(binary==0)=nan;
% 
% size(binary)
% figure
% imagesc(binary)
% area(binary(:,ROI)*lim(2),'LineStyle','none','FaceColor',[0.8 0.8 0.8])
% waitforbuttonpress
% 
% area(binary(:,ROI)*lim(2),'LineStyle','none','FaceColor',[0.8 0.8 0.8])
% hold on
% plot(handles.average(:,num_ROIs)), xlabel('Time (frame)'), ylabel('delF/Fo')
% value = get(handles.checkbox1, 'Value');
% if value == 1 && ROI~=1
%     plot(handles.average(:,1),'r')
% end
% hold off

guidata(hObject,handles)

% Second excel write
CoM=[handles.CoM(:,1) handles.CoM(:,2)]';
population_CoM=[mean(handles.CoM(:,1)); mean(handles.CoM(:,2))];
coordinates=[CoM population_CoM population_CoM];
x=[1:1:handles.number_of_ROIs];
excel=[x;coordinates;handles.average];

if length(num2str(handles.roi)) == 1
    ROI=['0' num2str(handles.roi)];
else
    ROI=num2str(handles.roi);
end

XLSfile=[num2str(handles.matname(1:(length(handles.matname)-4))) '-' ROI '-2.xlsx'];

if exist([handles.pathway XLSfile],'file')
    delete([handles.pathway XLSfile]);
    display(['over writing ' XLSfile])
end
xlswrite([handles.pathway XLSfile],excel)

handles.output=1;

guidata(hObject,handles)
close


%% --- Executes on RECLICK button press in pushbutton11.
function pushbutton11_Callback(hObject, eventdata, handles)

handles.output=2;
guidata(hObject,handles)
close


%% --- Executes on CUSTOMIZE button press in pushbutton7.
function pushbutton7_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles

axes(handles.axes1)

frame = getframe;
im = frame2im(frame);
imwrite(im,['process_snap.png'],'png');
display(['process_snap.png SNAPPED'])

imwrite(im,['process_snap.jpg'],'jpeg');
display(['process_snap.jpg SNAPPED'])

time_mask=zeros(handles.height,handles.width);
if isfield(handles,'spikes')
    
    for roi=1:handles.number_of_ROIs-2
        hit=find(handles.spikes(:,roi)==1);
        if roi==1 && ~isempty(hit)
            time_mask=time_mask+3*handles.ROI_mask(:,:,roi);
        elseif ~isempty(hit);
            time_mask=time_mask+2*handles.ROI_mask(:,:,roi);
        else
            time_mask=time_mask+handles.ROI_mask(:,:,roi);
        end
    end
    
    
    
    
    
    figure
%     pcolor([time_mask nan(handles.height,1); nan(1,handles.width+1)]);
%     shading flat;
%     set(gca, 'ydir', 'reverse');
    imagesc(time_mask)
    colormap([1 1 1; 0.8 0.8 0.8;1 0 0;0 0 1])
    caxis([0 3])
    axis equal
    axis off
%     hold on
%     
%     for i=1:handles.number_of_ROIs-2
%         if i==1
%             ROIboundary=handles.ROI_position{i};
%             plot(ROIboundary(:,2),ROIboundary(:,1),'k','LineWidth',1);
%         else
%             ROIboundary=handles.ROI_position{i};
%             plot(ROIboundary(:,2),ROIboundary(:,1),'k','LineWidth',1);
%         end
%     end
    
end

dummy.ROI_position=handles.ROI_position;
dummy.ROI_masks=handles.ROI_mask;
dummy.number_of_ROIs=handles.number_of_ROIs;
dummy.CoM=handles.CoM;

save(['step3.mat'],'dummy')

% hold off
% axes(handles.axes1)
% imagesc(handles.cell)
% colormap(jet)
% axis square
% axis off
% hold on
% for i=1:(handles.number_of_ROIs-2)
%     frame=find(handles.spikes(:,i)==1);
%     if nnz(frame)>0
%         colors=(frame(1)-10)/(handles.num_images);
%         ROIboundary=handles.ROI_position{i};
%         square=plot(ROIboundary(:,2),ROIboundary(:,1),'w','LineWidth',4);
%         square.Color=[colors 0 0];
%     end
%     
% end
