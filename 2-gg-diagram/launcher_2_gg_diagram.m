if (~exist('restart','var'))
    restart = true;
end

set(groot,'defaultAxesFontName','Formula1');

global background_color text_color

background_color =  [13/255, 17/255, 23/255];
text_color = [201,209,217]/255;


if restart
    clear all
    clc
    close all
    
    
    format long
    
    prefix = '/Users/juanmanzanero/Documents/software/fastest-lap';
    lib_suffix = 'dylib';
    
    addpath([prefix,'/src/main/matlab']);
    
    % (1) Load library
    if libisloaded('libfastestlapc')
        unloadlibrary libfastestlapc
    end
    
    loadlibrary([prefix,'/build/lib/libfastestlapc.',lib_suffix],[prefix,'/src/main/c/fastestlapc.h']);
    
    % (2) Load car
    vehicle = 'car';
    calllib("libfastestlapc","create_vehicle_from_xml",vehicle,[prefix,'/database/vehicles/f1/limebeer-2014-f1.xml']);
    
    % (3) Compute gg-diagrams
    
    % (3.1) Speed analysis
    fprintf('(1) Running speed analysis\n');
    n_points = 50;
    speeds = linspace(150.0, 300.0, 20);
    speed_analysis = [];
    for i = 1 : numel(speeds)
        fprintf('       %d- Speed: %d km/h...',i,speeds(i));
        [run.ay,run.ax_max,run.ax_min] = calllib("libfastestlapc",'gg_diagram',zeros(1,n_points),zeros(1,n_points),zeros(1,n_points),vehicle,speeds(i)/3.6, n_points);
        speed_analysis = [speed_analysis, run];
        fprintf(' completed!\n');
    end
    fprintf('\n\n');
    
    subplot(2,2,1)
    hold on
    for i = 1 : numel(speeds)
        plot(speed_analysis(i).ay,speed_analysis(i).ax_max,'-k');
        plot(speed_analysis(i).ay,speed_analysis(i).ax_min,'-k');
    end
    
    % (3.2) Brake bias analysis
    fprintf('(2) Running brake-bias analysis\n');
    
    brake_biases = [0.0,0.2,0.3,0.4,0.5,0.6,0.7,0.8,1.0];
    speed = 150.0;
    brake_bias_analysis = [];
    for i = 1 : numel(brake_biases)
        fprintf('       %d- Brake-bias: %f ...',i,brake_biases(i));
        calllib('libfastestlapc','vehicle_set_parameter',vehicle,'vehicle/chassis/brake_bias',brake_biases(i));
        [run.ay,run.ax_max,run.ax_min] = calllib("libfastestlapc",'gg_diagram',zeros(1,n_points),zeros(1,n_points),zeros(1,n_points),vehicle,speed/3.6, n_points);
        brake_bias_analysis = [brake_bias_analysis, run];
        fprintf(' completed!\n');
    end
    
    subplot(2,2,2)
    hold on
    for i = 1 : numel(brake_biases)
        plot(brake_bias_analysis(i).ay,brake_bias_analysis(i).ax_max,'-k');
        plot(brake_bias_analysis(i).ay,brake_bias_analysis(i).ax_min,'-k');
    end
    
    
    % (3.3) Horizontal cp position
    fprintf('(3) Running horizontal cp analysis\n');
    calllib('libfastestlapc','delete_variable',vehicle);
    calllib("libfastestlapc","create_vehicle_from_xml",vehicle,[prefix,'/database/vehicles/f1/limebeer-2014-f1.xml']);
    
    x_cp_values = linspace(-0.5,0.5,9);
    speed = 100.0;
    n_points = 70;
    x_cp_analysis = [];
    for i = 1 : numel(x_cp_values)
        fprintf('       %d- x_cp: %f ...',i,x_cp_values(i));
        calllib('libfastestlapc','vehicle_set_parameter',vehicle,'vehicle/chassis/pressure_center/x',x_cp_values(i));
        [run.ay,run.ax_max,run.ax_min] = calllib("libfastestlapc",'gg_diagram',zeros(1,n_points),zeros(1,n_points),zeros(1,n_points),vehicle,speed/3.6, n_points);
        x_cp_analysis = [x_cp_analysis, run];
        fprintf(' completed!\n');
    end
    
    subplot(2,2,3)
    hold on
    for i = 1 : numel(x_cp_values)
        plot(x_cp_analysis(i).ay,x_cp_analysis(i).ax_max,'-k');
        plot(x_cp_analysis(i).ay,x_cp_analysis(i).ax_min,'-k');
    end
    
    % (3.4) Vertical CoG position
    fprintf('(4) Running vertical CoG analysis\n');
    calllib('libfastestlapc','delete_variable',vehicle);
    calllib("libfastestlapc","create_vehicle_from_xml",vehicle,[prefix,'/database/vehicles/f1/limebeer-2014-f1.xml']);
    
    z_cog_values = linspace(-0.1,-0.4,5);
    speed = 150.0;
    n_points = 100;
    z_com_analysis = [];
    for i = 1 : numel(z_cog_values)
        fprintf('       %d- z_cog: %f ...',i,z_cog_values(i));
        calllib('libfastestlapc','vehicle_set_parameter',vehicle,'vehicle/chassis/com/z',z_cog_values(i));
        [run.ay,run.ax_max,run.ax_min] = calllib("libfastestlapc",'gg_diagram',zeros(1,n_points),zeros(1,n_points),zeros(1,n_points),vehicle,speed/3.6, n_points);
        z_com_analysis = [z_com_analysis, run];
        fprintf(' completed!\n');
    end
    
    subplot(2,2,4)
    hold on
    for i = 1 : numel(z_com_analysis)
        plot(z_com_analysis(i).ay,z_com_analysis(i).ax_max,'-k');
        plot(z_com_analysis(i).ay,z_com_analysis(i).ax_min,'-k');
    end
    
    restart = false;
end

% Make a video

n_frames = 5*30;

speed_analysis_ay = reshape([speed_analysis.ay],numel(speed_analysis(1).ay),numel(speed_analysis));
speed_analysis_ax_max = reshape([speed_analysis.ax_max],numel(speed_analysis(1).ax_max),numel(speed_analysis));
speed_analysis_ax_min = reshape([speed_analysis.ax_min],numel(speed_analysis(1).ax_min),numel(speed_analysis));

speed_analysis_ay = interp1(speeds, speed_analysis_ay', linspace(speeds(1),280.0,n_frames));
speed_analysis_ax_max = interp1(speeds, speed_analysis_ax_max', linspace(speeds(1),280.0,n_frames));
speed_analysis_ax_min = interp1(speeds, speed_analysis_ax_min', linspace(speeds(1),280.0,n_frames));


brake_bias_analysis_ay = reshape([brake_bias_analysis.ay],numel(brake_bias_analysis(1).ay),numel(brake_bias_analysis));
brake_bias_analysis_ax_max = reshape([brake_bias_analysis.ax_max],numel(brake_bias_analysis(1).ax_max),numel(brake_bias_analysis));
brake_bias_analysis_ax_min = reshape([brake_bias_analysis.ax_min],numel(brake_bias_analysis(1).ax_min),numel(brake_bias_analysis));

brake_bias_analysis_ay = interp1(brake_biases, brake_bias_analysis_ay', linspace(brake_biases(1),brake_biases(end),n_frames));
brake_bias_analysis_ax_max = interp1(brake_biases, brake_bias_analysis_ax_max', linspace(brake_biases(1),brake_biases(end),n_frames));
brake_bias_analysis_ax_min = interp1(brake_biases, brake_bias_analysis_ax_min', linspace(brake_biases(1),brake_biases(end),n_frames));

x_cp_analysis_ay = reshape([x_cp_analysis.ay],numel(x_cp_analysis(1).ay),numel(x_cp_analysis));
x_cp_analysis_ax_max = reshape([x_cp_analysis.ax_max],numel(x_cp_analysis(1).ax_max),numel(x_cp_analysis));
x_cp_analysis_ax_min = reshape([x_cp_analysis.ax_min],numel(x_cp_analysis(1).ax_min),numel(x_cp_analysis));

x_cp_analysis_ay = interp1(x_cp_values, x_cp_analysis_ay', linspace(x_cp_values(1),x_cp_values(end),n_frames));
x_cp_analysis_ax_max = interp1(x_cp_values, x_cp_analysis_ax_max', linspace(x_cp_values(1),x_cp_values(end),n_frames));
x_cp_analysis_ax_min = interp1(x_cp_values, x_cp_analysis_ax_min', linspace(x_cp_values(1),x_cp_values(end),n_frames));

z_com_analysis_ay = reshape([z_com_analysis.ay],numel(z_com_analysis(1).ay),numel(z_com_analysis));
z_com_analysis_ax_max = reshape([z_com_analysis.ax_max],numel(z_com_analysis(1).ax_max),numel(z_com_analysis));
z_com_analysis_ax_min = reshape([z_com_analysis.ax_min],numel(z_com_analysis(1).ax_min),numel(z_com_analysis));

z_com_analysis_ay = interp1(z_cog_values, z_com_analysis_ay', linspace(z_cog_values(1),z_cog_values(end),n_frames));
z_com_analysis_ax_max = interp1(z_cog_values, z_com_analysis_ax_max', linspace(z_cog_values(1),z_cog_values(end),n_frames));
z_com_analysis_ax_min = interp1(z_cog_values, z_com_analysis_ax_min', linspace(z_cog_values(1),z_cog_values(end),n_frames));

for i = 1 %: n_frames
    i
    h=figure('Visible','off');
    h.Color = background_color;
    subplot(2,2,1)
    hold all
    speed_analysis_ax_min(:,1) = speed_analysis_ax_min(:,2);
    plot_data(speed_analysis_ay, speed_analysis_ax_max, speed_analysis_ax_min,3,i);
    header_data = linspace(speeds(1),280.0,n_frames);
    %title(['Velocity = ',num2str(header_data(i),'%.0f'),'km/h'],'Color',text_color)
    ylabel('longitudinal acceleration [g]');
    xlabel('lateral acceleration [g]');
    subplot(2,2,2)
    hold all
    brake_bias_analysis_ax_min(:,1) = brake_bias_analysis_ax_min(:,2);
    plot_data(brake_bias_analysis_ay, brake_bias_analysis_ax_max, brake_bias_analysis_ax_min,4,i);
    header_data = linspace(brake_biases(1),brake_biases(end),n_frames);
    title(['Brake-bias = ',num2str(header_data(i)*100,'%.0f'),'% front'],'Color',text_color)
    
    subplot(2,2,3)
    hold all
    x_cp_analysis_ax_min(:,1) = x_cp_analysis_ax_min(:,2);
    plot_data(x_cp_analysis_ay, x_cp_analysis_ax_max, x_cp_analysis_ax_min,3,i);
    header_data = (1.6+linspace(x_cp_values(1),x_cp_values(end),n_frames))/(1.6+1.8);
    title([num2str(header_data(i)*100,'%.0f'),'% Front wing, ',num2str(100-header_data(i)*100,'%.0f'),'% Rear wing'],'Color',text_color)
    ylabel('longitudinal acceleration [g]');
    xlabel('lateral acceleration [g]')
    subplot(2,2,4)
    hold all
    z_com_analysis_ax_min(:,1) = z_com_analysis_ax_min(:,2);
    plot_data(z_com_analysis_ay, z_com_analysis_ax_max, z_com_analysis_ax_min,4,i);
    header_data = linspace(z_cog_values(1),z_cog_values(end),n_frames);
    title(['Center of mass height = ',num2str(-header_data(i),'%.2f'),'m'],'Color',text_color)
    xlabel('lateral acceleration [g]')
    set(h, 'InvertHardcopy', 'off');
    print(h,['figs/fig_',num2str(i)],'-dpng','-r300');
end

function plot_data(data_ay,data_ax_max,data_ax_min,a_grid,i_frame)
global background_color text_color

ax=gca;
ax.Color = background_color;
ax.GridColor = text_color;
ax.XColor = text_color;
ax.YColor = text_color;
plot(data_ay(i_frame,:)/9.81,data_ax_max(i_frame,:)/9.81,'Color',text_color,'LineWidth',2)
plot(data_ay(i_frame,:)/9.81,data_ax_min(i_frame,:)/9.81,'Color',text_color,'LineWidth',2)
plot(-data_ay(i_frame,:)/9.81,data_ax_max(i_frame,:)/9.81,'Color',text_color,'LineWidth',2)
plot(-data_ay(i_frame,:)/9.81,data_ax_min(i_frame,:)/9.81,'Color',text_color,'LineWidth',2)
xlim([-a_grid,a_grid]);
ylim([-a_grid,a_grid]);
xticks(-a_grid:a_grid);
yticks(-a_grid:a_grid);
box on
grid on
end
