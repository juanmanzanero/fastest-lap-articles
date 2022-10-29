clear all

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
calllib(fastest_lap,'create_track_from_xml',circuit,'C:\Users\jmanz\Documents\software\fastest-lap-projects\2022\Suzuka\0_Preprocessor\suzuka_uniform_2000.xml');
n_points = 2000;
s = calllib(fastest_lap,'track_download_data',zeros(1,n_points),circuit,n_points,'arclength');

% Load lap without drs
run_without_drs = load('../3-simulation-without-drs/run.mat');

% Load lap with drs
run_with_drs = load('./run.mat');

% Load telemetry lap
run_telemetry = load('../1_telemetry_comparison/data.mat');


h = figure('Color',background_color);
if ismac
    h.Position = [196   471   975   226];
elseif ispc
    h.Position = [196         383        1620         314];
end
plot(run_telemetry.s_ref+40,run_telemetry.speed_ref,'LineWidth',2)
hold on
plot(s,run_without_drs.run.u*3.6,'LineWidth',2)
plot(s,run_with_drs.run.u*3.6,'LineWidth',2)
xlim([2500,5800]);

title('Simulation - last section of Suzuka','Color',text_color)
set(gca,'FontSize',20);

ax = h.CurrentAxes;
ylim([50,350])
yticks(50:25:350);
xticks([2500:125:5800])

ax.YTickLabel = cellfun(@(f)([f,'km/h']),ax.YTickLabel,'UniformOutput',false);
ax.XTickLabel = cellfun(@(f)([f,'m']),ax.XTickLabel,'UniformOutput',false);

ax.YTickLabel(2:2:end) = {''};
ax.XTickLabel(2:4:end) = {''};
ax.XTickLabel(3:4:end) = {''};
ax.XTickLabel(4:4:end) = {''};
ax.XTickLabelRotation = 0;
ax.Color = background_color;
ax.XColor = text_color;
ax.YColor = text_color;
legend({'Telemetry (fastf1)','Simulation','Simulation with DRS'},'AutoUpdate','off','Location','southwest','TextColor',text_color,'Color',background_color)
patch([4900,5100,5100,4900],[60,60,315,315],[1,0,0],'FaceAlpha',0.2,'LineStyle','none');
text(5000,260,'130R','Color',text_color,'horizontalAlignment','center','FontName',font_name,'FontSize',20)
grid on
box on
set(h, 'InvertHardcopy', 'off')
print(h,'velocity_plot.png','-dpng','-r300')

% Plot velocity


function clean(fastest_lap)

if libisloaded(fastest_lap)
    unloadlibrary(fastest_lap)
end

end