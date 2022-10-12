clear all
close all
fastest_lap = 'flap';

if ismac
    font_name = 'Formula1';
elseif ispc
    font_name = 'Formula1 Display Regular';
end

background_color =  [13/255, 17/255, 23/255];
text_color = [201,209,217]/255;
blue   = [0   0.447000000000000   0.741];
set(groot,'defaultAxesFontName',font_name);

if ismac
    fastest_lap_path = '/Users/juanmanzanero/Documents/software/fastest-lap/';
    fastest_lap_library_path = [fastest_lap_path,'build/lib/'];
    fastest_lap_version = '';
    fastest_lap_suffix = 'dylib';
elseif ispc
    fastest_lap_path = 'C:\Users\jmanz\Documents\software\fastest-lap-vs\fastest-lap';
    fastest_lap_library_path = 'C:\Users\jmanz\Documents\software\fastest-lap-vs\vs\x64\Release';
    fastest_lap_version = '-0.4';
    fastest_lap_suffix = 'dll';
end

addpath([fastest_lap_path, '/src/main/matlab']);

dtor = onCleanup(@()(clean(fastest_lap)));

loadlibrary([fastest_lap_library_path,filesep,'libfastestlapc',fastest_lap_version,'.',fastest_lap_suffix],...
    [fastest_lap_path,'/src/main/c/fastestlapc.h'],'alias',fastest_lap);

% Load Suzuka circuit
circuit = 'suzuka';
calllib(fastest_lap,'create_track_from_xml',circuit,'suzuka_uniform_2000.xml');
n_points = 2000;
s = calllib(fastest_lap,'track_download_data',zeros(1,n_points),circuit,n_points,'arclength');

    i_drs_start = 1450;
    i_drs_end   = 1820;


x = calllib(fastest_lap,'track_download_data',zeros(1,n_points),circuit,n_points,'centerline.x');
y = calllib(fastest_lap,'track_download_data',zeros(1,n_points),circuit,n_points,'centerline.y');
kappa = calllib(fastest_lap,'track_download_data',zeros(1,n_points),circuit,n_points,'curvature');
h = figure('Color',background_color,'Position',[137         125        1014         572]);
hold on
plot(x(i_drs_start:i_drs_end),y(i_drs_start:i_drs_end),'LineWidth',14,'Color',[0.8500    0.3250    0.0980])
plot(x(1:200),y(1:200),'LineWidth',14,'Color',[0.9290    0.6940    0.1250])
plot(x,y,'Color',text_color,'LineWidth',4);
annotation('textbox',[0.65,0.47,0.4,0.4],'String','Official DRS zone','HorizontalAlignment','center','VerticalAlignment','middle','FontName',font_name,'FontSize',25,'Color',[0.9290    0.6940    0.1250],'LineStyle','none')
annotation('textbox',[0.05,0.35,0.4,0.4],'String','New DRS zone','HorizontalAlignment','center','VerticalAlignment','middle','FontName',font_name,'FontSize',25,'Color',[0.8500    0.3250    0.0980],'LineStyle','none')
ax = h.CurrentAxes;
ax.Visible = 'off';
ax.Color = background_color;
ax.XColor = text_color;
ax.YColor = text_color;
axis equal
set(h, 'InvertHardcopy', 'off')
print(h,'new_drs_zone.png','-dpng','-r300')
function clean(fastest_lap)

if libisloaded(fastest_lap)
    unloadlibrary(fastest_lap)
end

end