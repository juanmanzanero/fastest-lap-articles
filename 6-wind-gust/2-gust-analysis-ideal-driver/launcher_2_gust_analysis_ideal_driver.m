global fastest_lap

rerun = true;

if rerun
    fastest_lap = 'flap';
    track_file = 'cota_uniform_1000.xml';
    vehicle_file = 'neutral-car.xml';

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
    else
        fastest_lap_path = '/home/r660/r660391/storage/fastest-lap/';
        fastest_lap_library_path = [fastest_lap_path,'build/lib/'];
        fastest_lap_version = '';
        fastest_lap_suffix = 'so';
    end

    addpath([fastest_lap_path,'/src/main/matlab']);

    % (1) Load library
    if libisloaded(fastest_lap)
        unloadlibrary(fastest_lap)
    end

    loadlibrary([fastest_lap_library_path,filesep,'libfastestlapc',fastest_lap_version,'.',fastest_lap_suffix],...
        [fastest_lap_path,'/src/main/c/fastestlapc.h'],'alias',fastest_lap);

    calllib(fastest_lap,'set_print_level',2);

    % (2) Load circuit
    circuit_name = 'circuit';
    calllib(fastest_lap,'create_track_from_xml',circuit_name, track_file);

    % (2.1) Get circuit data
    n_points = calllib(fastest_lap, 'track_download_number_of_points', circuit_name);
    track_length = calllib(fastest_lap, 'track_download_length', circuit_name);
    s = calllib(fastest_lap, 'track_download_data',zeros(1,n_points), circuit_name, n_points, 'arclength');

    x_left = calllib(fastest_lap,'track_download_data',zeros(1,n_points), circuit_name, n_points,'left.x');
    y_left = calllib(fastest_lap,'track_download_data',zeros(1,n_points), circuit_name, n_points,'left.y');
    x_right = calllib(fastest_lap,'track_download_data',zeros(1,n_points), circuit_name, n_points,'right.x');
    y_right = calllib(fastest_lap,'track_download_data',zeros(1,n_points), circuit_name, n_points,'right.y');
    x_center = calllib(fastest_lap,'track_download_data',zeros(1,n_points), circuit_name, n_points,'centerline.x');
    y_center = calllib(fastest_lap,'track_download_data',zeros(1,n_points), circuit_name, n_points,'centerline.y');
    theta    = calllib(fastest_lap,'track_download_data',zeros(1,n_points), circuit_name, n_points,'heading-angle');
    nl    = calllib(fastest_lap,'track_download_data',zeros(1,n_points), circuit_name, n_points,'distance-left-boundary');
    nr    = calllib(fastest_lap,'track_download_data',zeros(1,n_points), circuit_name, n_points,'distance-right-boundary');

    % (3) Load car
    vehicle_name = 'car';
    calllib(fastest_lap, 'create_vehicle_from_xml', vehicle_name, vehicle_file);

    % (4) Load optimal laptime for the full circuit
    simulation_data = load('../1-nominal-trajectory/full_lap').simulation_data;

    % (5) Run the corner with wind
    calllib(fastest_lap,'set_print_level',0)
    wind_angles = linspace(0,2*pi,200);

    corner_study = struct('s_start', 2300, 's_finish', 2800, 'n_points', 200);
    [~,corner_study.i_start] = min(abs(s-corner_study.s_start));
    [~,corner_study.i_finish] = min(abs(s-corner_study.s_finish));
    corner_study.s = linspace(s(corner_study.i_start), s(corner_study.i_finish), corner_study.n_points);

    % (5.1) Write the initial condition
    corner_study.q_start = [simulation_data.("front-axle.left-tire.kappa")(corner_study.i_start), ...
        simulation_data.("front-axle.right-tire.kappa")(corner_study.i_start), ...
        simulation_data.("rear-axle.left-tire.kappa")(corner_study.i_start), ...
        simulation_data.("rear-axle.right-tire.kappa")(corner_study.i_start), ...
        simulation_data.("chassis.velocity.x")(corner_study.i_start), ...
        simulation_data.("chassis.velocity.y")(corner_study.i_start), ...
        simulation_data.("chassis.omega.z")(corner_study.i_start), ...
        0.0, ...
        simulation_data.("road.lateral-displacement")(corner_study.i_start), ...
        simulation_data.("road.track-heading-angle")(corner_study.i_start)];

    corner_study.qa_start = [simulation_data.("chassis.Fz_fl")(corner_study.i_start), ...
        simulation_data.("chassis.Fz_fr")(corner_study.i_start), ...
        simulation_data.("chassis.Fz_rl")(corner_study.i_start), ...
        simulation_data.("chassis.Fz_rr")(corner_study.i_start)];

    corner_study.u_start = [simulation_data.("front-axle.steering-angle")(corner_study.i_start), ...
        simulation_data.("rear-axle.boost")(corner_study.i_start), ...
        simulation_data.("chassis.throttle")(corner_study.i_start), ...
        simulation_data.("chassis.brake-bias")(corner_study.i_start)];


    for i_wind = 1:numel(wind_angles)

        vehicle_copy = 'vehicle_copy';
        calllib(fastest_lap,'copy_variable',vehicle_name, vehicle_copy);
        northward_wind_intensity = 50*cos(wind_angles(i_wind))/3.6;
        eastward_wind_intensity  = 50*sin(wind_angles(i_wind))/3.6;
        %        s_wind_start = 2524.926834327442; s_wind_end = 2645.947292617647;
        s_wind_start = 2400.0; s_wind_end   = 2700.0;
        wind_mesh = [0,s_wind_start, s_wind_start + 10, s_wind_end - 10, s_wind_end, track_length];
        calllib(fastest_lap, "vehicle_declare_new_variable_parameter", vehicle_copy, 'vehicle/chassis/aerodynamics/wind_velocity/northward', 'no_north_wind;full_north_wind',  2, [0,northward_wind_intensity], numel(wind_mesh), [0,0,1,1,0,0], wind_mesh);
        calllib(fastest_lap, "vehicle_declare_new_variable_parameter", vehicle_copy, 'vehicle/chassis/aerodynamics/wind_velocity/eastward', 'no_east_wind;full_east_wind',  2, [0,eastward_wind_intensity], numel(wind_mesh), [0,0,1,1,0,0], wind_mesh);

        calllib(fastest_lap,'create_vector', 'initial_condition/state', numel(corner_study.q_start), corner_study.q_start);
        calllib(fastest_lap,'create_vector', 'initial_condition/algebraic_state', numel(corner_study.qa_start), corner_study.qa_start);
        calllib(fastest_lap,'create_vector', 'initial_condition/control', numel(corner_study.u_start), corner_study.u_start);
        calllib(fastest_lap, 'vehicle_set_parameter', vehicle_name, 'vehicle/chassis/brake_bias', simulation_data.("chassis.brake-bias")(corner_study.i_start));

        % (5.1) Write options
        options =          '<options>';
        options = [options,    '<closed_simulation> false </closed_simulation>'];
        options = [options,    '<initial_condition>'];
        options = [options,    '    <q  from_table="initial_condition/state"/>'];
        options = [options,    '    <qa from_table="initial_condition/algebraic_state"/>'];
        options = [options,    '    <u  from_table="initial_condition/control"/>'];
        options = [options,    '</initial_condition>'];
        options = [options,    '<output_variables>'];
        options = [options,    '    <prefix> run_corner/ </prefix>'];
        options = [options,    '</output_variables>'];
        options = [options,    '<sigma> 0.5 </sigma>'];
        options = [options,'</options>'];

        % (5.2) Run
        calllib(fastest_lap,'optimal_laptime', vehicle_copy, circuit_name, numel(corner_study.s), corner_study.s, options);

        % (5.3) Download data
        corner_study.simulation_data{i_wind} = get_run_data('run_corner/', numel(corner_study.s));

        % (5.4) Cleanup
        calllib(fastest_lap,'delete_variable','run_corner/*');
        calllib(fastest_lap,'delete_variable','initial_condition/*');
        calllib(fastest_lap,'delete_variable',vehicle_copy);

        % (5.5) Display stats
        fprintf('Wind angle: %f       Delta laptime: %f\n',rad2deg(wind_angles(i_wind)), corner_study.simulation_data{i_wind}.time(end) - 9.362692319028884);
    end

    save('autosave_longer_gust')

else
    load('autosave_better_gust')
end

calm_simulation = load('../1-nominal-trajectory/air_in_calm_simulation.mat');

plot_figures = false;

if plot_figures
    %
    % Plotting
    %
    background_color =  [13/255, 17/255, 23/255];
    text_color = [201,209,217]/255;
    blue   = [0   0.447000000000000   0.741];

    if ismac
        font_name = 'Formula1';
    else
        font_name = 'Formula1 Display Regular';
    end
    set(groot,'defaultAxesFontName',font_name);

    % % (1) Plot delta time vs wind angle
    delta_time = zeros(1,numel(wind_angles));
    for i = 1 : numel(wind_angles)
        delta_time(i) = corner_study.simulation_data{i}.time(end) - 9.362692319028884;
    end
    delta_time(1) = delta_time(end);
    % h = figure('Position',[546   381   838   597]);
    % h.Color = background_color;
    % delta_time_pos = delta_time;
    % delta_time_pos(delta_time_pos < 0) = NaN;
    % nan_position = find(isnan(delta_time_pos));
    % delta_time_pos(nan_position(1)) = delta_time(nan_position(1));
    % delta_time_pos(nan_position(end)) = delta_time(nan_position(end));
    %
    % delta_time_neg = delta_time;
    % delta_time_neg(delta_time_pos >= 0) = NaN;
    %
    % plot(rad2deg(wind_angles), delta_time_pos,'Color','r','LineWidth',2)
    % hold on
    % plot(rad2deg(wind_angles), delta_time_neg,'Color','g','LineWidth',2)
    % ax = h.CurrentAxes;
    % ax.Color = background_color;
    % ax.XColor = text_color;
    % ax.YColor = text_color;
    % xlim([0,360]);
    % ax.XTick = 0:45:360;
    % ax.XTickLabel = {'N','NE','E','SE','S','SW','W','NW','N'};
    %
    % ax.YTick = -0.8:0.2:0.6;
    % ax.YTickLabel = cellfun(@(f)([num2str(str2num(f),'%.3f'),'s']), ax.YTickLabel,'UniformOutput',false);
    % ax.Position(4) = ax.Position(3);
    % grid on
    % box on
    % ax.FontSize=15;
    % t = title('delta-time with respect to air in calm','Color',text_color,'FontSize',20);
    % t.Position(2) = t.Position(2) + 0.05;
    %
    % a_faster = annotation('textbox',[0,0,0.2,0.1],'String','Faster','Color',text_color,'LineStyle','none','FontName',font_name,'FontSize',20,'Position',[0.386634844868736   0.278056951423786   0.200000000000001   0.100000000000000]);
    % a_slower = annotation('textbox',[0,0.1,0.2,0.1],'String','Slower','Color',text_color,'LineStyle','none','FontName',font_name,'FontSize',20,'Position',[0.563245823389023   0.672864321608041   0.200000000000000   0.100000000000000]);
    % xlabel('gust direction')
    %
    % [im,~,al] = imread('Simple_compass_rose.svg.png');
    %
    % ax_compass = axes('Position',[0.2,0.5,0.35,0.35]);
    %
    % h_im = imagesc((im));
    % h_im.AlphaData = al;
    % cmap = [1 1 1 %// light gray
    %         0  0  0] %// white
    % colormap(cmap)
    % ax_compass.Visible = 'off';
    % axis equal
    %
    % set(h, 'InvertHardcopy', 'off')
    % print(h,'delta_time_wrt_no_gust','-dpng','-r300')
    %
    % % (2) Plot telemetry comparison for the worst case
    % [~,i_worst] = max(delta_time);
    %
    % h = figure('Position',[458          97        1233         881]);
    % h.Color = background_color;
    %
    % subplot(6,1,1)
    % plot(calm_simulation.corner_study.simulation_data.("road.arclength"),calm_simulation.corner_study.simulation_data.("chassis.velocity.x")*3.6,'LineWidth',2);
    % hold on
    % plot(corner_study.simulation_data{i_worst}.("road.arclength"),corner_study.simulation_data{i_worst}.("chassis.velocity.x")*3.6,'LineWidth',2);
    % t=title('velocity profiles [km/h]','Color',text_color)
    % xlim([2300,2800])
    % ax = gca;
    % ax.Color = background_color;
    % ax.XColor = text_color;
    % ax.YColor = text_color;
    % ax.XTickLabel = cellfun(@(f)(''),ax.XTickLabel,'UniformOutput',false);
    % ax.YTickMode = 'manual';
    % ax.YTickLabel = {'100km/h','','200km/h','','300km/h'};
    % ax.FontSize = 15;
    % ax.Position(4) = 0.102;
    % ylim([50,300])
    % grid on
    % legend({'air in calm','worst gust direction'},'location','southwest','TextColor',text_color)
    %
    % subplot(6,1,2)
    % plot(calm_simulation.corner_study.simulation_data.("road.arclength"),max(0,calm_simulation.corner_study.simulation_data.("chassis.throttle"))*100,'LineWidth',2);
    % hold on
    % plot(corner_study.simulation_data{i_worst}.("road.arclength"),max(0,corner_study.simulation_data{i_worst}.("chassis.throttle"))*100,'LineWidth',2);
    % t=title('throttle','Color',text_color)
    % xlim([2300,2800])
    % ax = gca;
    % ax.Color = background_color;
    % ax.XColor = text_color;
    % ax.YColor = text_color;
    % ax.XTickLabel = cellfun(@(f)(''),ax.XTickLabel,'UniformOutput',false);
    % ax.FontSize = 15;
    % ax.Position(4) = 0.102;;
    % ax.YTickMode = 'manual';
    % ax.YTickLabel = cellfun(@(f)([f,'%']),ax.YTickLabel,'UniformOutput',false);
    % grid on
    %
    %
    % subplot(6,1,3)
    % plot(calm_simulation.corner_study.simulation_data.("road.arclength"),corner_study.simulation_data{i_worst}.time - calm_simulation.corner_study.simulation_data.time,'Color',text_color,'LineWidth',2)
    % t=title('delta-time','Color',text_color)
    % xlim([2300,2800])
    % ax = gca;
    % ax.Color = background_color;
    % ax.XColor = text_color;
    % ax.YColor = text_color;
    % ax.XTickLabel = cellfun(@(f)(''),ax.XTickLabel,'UniformOutput',false);
    % ax.FontSize = 15;
    % ax.Position(4) = 0.102;
    % grid on
    %
    % subplot(6,1,4)
    % plot(calm_simulation.corner_study.simulation_data.("road.arclength"),(corner_study.simulation_data{i_worst}.("chassis.aerodynamics.lift") - calm_simulation.corner_study.simulation_data.("chassis.aerodynamics.lift"))/(795*9.81),'Color',text_color,'LineWidth',2)
    % t=title('delta-downforce [g]','Color',text_color)
    % xlim([2300,2800])
    % ax = gca;
    % ax.Color = background_color;
    % ax.XColor = text_color;
    % ax.YColor = text_color;
    % ax.XTickLabel = cellfun(@(f)(''),ax.XTickLabel,'UniformOutput',false);
    % ax.FontSize = 15;
    % ylim([-0.6,0])
    % ax.Position(4) = 0.102;
    % grid on
    %
    % subplot(6,1,5)
    % plot(calm_simulation.corner_study.simulation_data.("road.arclength"),(abs(corner_study.simulation_data{i_worst}.("chassis.aerodynamics.drag.x")) - abs(calm_simulation.corner_study.simulation_data.("chassis.aerodynamics.drag.x")))/(795*9.81),'Color',text_color,'LineWidth',2)
    % t=title('delta-drag [g]','Color',text_color);
    % xlim([2300,2800])
    % ax = gca;
    % ax.Color = background_color;
    % ax.XColor = text_color;
    % ax.YColor = text_color;
    % ax.XTickLabel = cellfun(@(f)(''),ax.XTickLabel,'UniformOutput',false);
    % ax.FontSize = 15;
    %
    % ylim([-0.15,0.0])
    % ax.Position(4) = 0.102;
    % grid on
    %
    % subplot(6,1,6)
    % plot(calm_simulation.corner_study.simulation_data.("road.arclength"),(corner_study.simulation_data{i_worst}.("chassis.aerodynamics.drag.y") - calm_simulation.corner_study.simulation_data.("chassis.aerodynamics.drag.y"))/(795*9.81),'Color',text_color,'LineWidth',2)
    % t=title('delta-side-force [g]','Color',text_color);
    % xlim([2300,2800])
    % ax = gca;
    % ax.Color = background_color;
    % ax.XColor = text_color;
    % ax.YColor = text_color;
    % ax.FontSize = 15;
    % ax.XTickLabel = cellfun(@(f)([f,'m']),ax.XTickLabel,'UniformOutput',false);
    % ylim([-0.06,0.06])
    % ax.Position(4) = 0.102;
    % grid on
    %
    % annotation('textbox',[0,0.95,1.0,0.05],'String','Detailed analysis of the worst gust direction.','LineStyle','none','FontSize',20,'Color',text_color,'FontName',font_name,'HorizontalAlignment','center','VerticalAlignment','middle');
    %
    % set(h, 'InvertHardcopy', 'off')
    % print(h,'detail_worst_gust_direction','-dpng','-r300')
    %
    %
    % % (3) Plot telemetry comparison for the best case
    [~,i_best] = min(delta_time);
    %
    % h = figure('Position',[458          97        1233         881]);
    % h.Color = background_color;
    %
    % subplot(6,1,1)
    % plot(calm_simulation.corner_study.simulation_data.("road.arclength"),calm_simulation.corner_study.simulation_data.("chassis.velocity.x")*3.6,'LineWidth',2);
    % hold on
    % plot(corner_study.simulation_data{i_best}.("road.arclength"),corner_study.simulation_data{i_best}.("chassis.velocity.x")*3.6,'LineWidth',2);
    % t=title('velocity profiles [km/h]','Color',text_color)
    % xlim([2300,2800])
    % ax = gca;
    % ax.Color = background_color;
    % ax.XColor = text_color;
    % ax.YColor = text_color;
    % ax.XTickLabel = cellfun(@(f)(''),ax.XTickLabel,'UniformOutput',false);
    % ax.YTickMode = 'manual';
    % ax.YTickLabel = {'100km/h','','200km/h','','300km/h'};
    % ax.FontSize = 15;
    % ax.Position(4) = 0.102;
    % ylim([50,300])
    % grid on
    % legend({'air in calm','worst gust direction'},'location','southwest','TextColor',text_color)
    %
    % subplot(6,1,2)
    % plot(calm_simulation.corner_study.simulation_data.("road.arclength"),max(0,calm_simulation.corner_study.simulation_data.("chassis.throttle"))*100,'LineWidth',2);
    % hold on
    % plot(corner_study.simulation_data{i_best}.("road.arclength"),max(0,corner_study.simulation_data{i_best}.("chassis.throttle"))*100,'LineWidth',2);
    % t=title('throttle','Color',text_color)
    % xlim([2300,2800])
    % ax = gca;
    % ax.Color = background_color;
    % ax.XColor = text_color;
    % ax.YColor = text_color;
    % ax.XTickLabel = cellfun(@(f)(''),ax.XTickLabel,'UniformOutput',false);
    % ax.FontSize = 15;
    % ax.Position(4) = 0.102;;
    % ax.YTickMode = 'manual';
    % ax.YTickLabel = cellfun(@(f)([f,'%']),ax.YTickLabel,'UniformOutput',false);
    % grid on
    %
    %
    % subplot(6,1,3)
    % plot(calm_simulation.corner_study.simulation_data.("road.arclength"),corner_study.simulation_data{i_best}.time - calm_simulation.corner_study.simulation_data.time,'Color',text_color,'LineWidth',2)
    % t=title('delta-time','Color',text_color)
    % xlim([2300,2800])
    % ax = gca;
    % ax.Color = background_color;
    % ax.XColor = text_color;
    % ax.YColor = text_color;
    % ax.XTickLabel = cellfun(@(f)(''),ax.XTickLabel,'UniformOutput',false);
    % ax.FontSize = 15;
    % ax.Position(4) = 0.102;
    % grid on
    %
    % subplot(6,1,4)
    % plot(calm_simulation.corner_study.simulation_data.("road.arclength"),(corner_study.simulation_data{i_best}.("chassis.aerodynamics.lift") - calm_simulation.corner_study.simulation_data.("chassis.aerodynamics.lift"))/(795*9.81),'Color',text_color,'LineWidth',2)
    % t=title('delta-downforce [g]','Color',text_color)
    % xlim([2300,2800])
    % ax = gca;
    % ax.Color = background_color;
    % ax.XColor = text_color;
    % ax.YColor = text_color;
    % ax.XTickLabel = cellfun(@(f)(''),ax.XTickLabel,'UniformOutput',false);
    % ax.FontSize = 15;
    % ylim([0,1.0])
    % ax.Position(4) = 0.102;
    % grid on
    %
    % subplot(6,1,5)
    % plot(calm_simulation.corner_study.simulation_data.("road.arclength"),(abs(corner_study.simulation_data{i_best}.("chassis.aerodynamics.drag.x")) - abs(calm_simulation.corner_study.simulation_data.("chassis.aerodynamics.drag.x")))/(795*9.81),'Color',text_color,'LineWidth',2)
    % t=title('delta-drag [g]','Color',text_color);
    % xlim([2300,2800])
    % ax = gca;
    % ax.Color = background_color;
    % ax.XColor = text_color;
    % ax.YColor = text_color;
    % ax.XTickLabel = cellfun(@(f)(''),ax.XTickLabel,'UniformOutput',false);
    % ax.FontSize = 15;
    %
    % ylim([0.0,0.3])
    % ax.Position(4) = 0.102;
    % grid on
    %
    % subplot(6,1,6)
    % plot(calm_simulation.corner_study.simulation_data.("road.arclength"),(corner_study.simulation_data{i_best}.("chassis.aerodynamics.drag.y") - calm_simulation.corner_study.simulation_data.("chassis.aerodynamics.drag.y"))/(795*9.81),'Color',text_color,'LineWidth',2)
    % t=title('delta-side-force [g]','Color',text_color);
    % xlim([2300,2800])
    % ax = gca;
    % ax.Color = background_color;
    % ax.XColor = text_color;
    % ax.YColor = text_color;
    % ax.FontSize = 15;
    % ax.XTickLabel = cellfun(@(f)([f,'m']),ax.XTickLabel,'UniformOutput',false);
    % ylim([-0.1,0.1])
    % ax.Position(4) = 0.102;
    % grid on
    %
    % annotation('textbox',[0,0.95,1.0,0.05],'String','Detailed analysis of the best gust direction.','LineStyle','none','FontSize',20,'Color',text_color,'FontName',font_name,'HorizontalAlignment','center','VerticalAlignment','middle');
    %
    % set(h, 'InvertHardcopy', 'off')
    % print(h,'detail_best_gust_direction','-dpng','-r300')


    % Write a video
    corner_study.simulation_data{i_best}.x_center = interp1(s(corner_study.i_start:corner_study.i_finish),x_center(corner_study.i_start:corner_study.i_finish),corner_study.s)';
    corner_study.simulation_data{i_best}.y_center = interp1(s(corner_study.i_start:corner_study.i_finish),y_center(corner_study.i_start:corner_study.i_finish),corner_study.s)';
    draw_frame(corner_study.simulation_data{i_best}, readstruct('neutral-car.xml'), s, x_left, x_right, y_left, y_right, x_center, y_center, theta, nl, nr, [s_wind_start,s_wind_end],[sin(wind_angles(i_best)),cos(wind_angles(i_best))], 100);

end


function run_data = get_run_data(prefix, n_points)
% Generate a table with all the outputs available from the model

global fastest_lap

n_max_char = 120;

% Get car model metadata
[n_state,n_algebraic,n_control,n_outputs] = calllib(fastest_lap,'vehicle_type_get_sizes',0,0,0,0,'f1-3dof');
[key_name, state_names, algebraic_state_names, control_names, output_names] = calllib(fastest_lap,'vehicle_type_get_names',blanks(n_max_char),repmat({blanks(n_max_char)},1,n_state),repmat({blanks(n_max_char)},1,n_algebraic),repmat({blanks(n_max_char)},1,n_control),repmat({blanks(n_max_char)},1,n_outputs), n_max_char, 'f1-3dof');

all_names = [{key_name}, state_names(:)', algebraic_state_names(:)', control_names(:)', output_names(:)'];

% Get all the variables available
variable_data = zeros(n_points, numel(all_names));
counter = 1;
for name_ = all_names
    name = name_{:};
    variable_data(:,counter) = calllib(fastest_lap, "download_vector", zeros(1,n_points), n_points, [prefix,name]);
    counter = counter + 1;
end

variable_data = [variable_data,repmat(calllib(fastest_lap, "download_scalar", [prefix,'laptime']),n_points,1)];
run_data = array2table(variable_data,'VariableNames', [all_names,{'laptime'}]);

end

function draw_frame(simulation_data, vehicle_data, s_track,x_left, x_right, y_left, y_right, x_center, y_center, theta, nl, nr, s_wind_delimiters, wind, i_frame)

% (1) Plot car and track
x = simulation_data.("chassis.position.x");
y = simulation_data.("chassis.position.y");

screen_size = get(0,'ScreenSize');
h = figure('Position',[0 0 screen_size(3) 1080.0/1920.0*screen_size(3)]);
hold on;

% (1) Plot track
patch([x_left, x_right], [y_left, y_right], [13/255, 17/255, 23/255]);

axis equal
h.CurrentAxes.Visible = 'off';
ax = gca;
ax.Position = [0,0,1,1];

% (1.1) Plot the wind in the track
[~,i_wind_start] = min(abs(s_track - s_wind_delimiters(1)));
[~,i_wind_finish] = min(abs(s_track - s_wind_delimiters(2)));

s_wind = s_track(i_wind_start:i_wind_finish);
s_wind_sample = s_wind(1):5:s_wind(end);

x_center_wind = interp1(s_wind, x_center(i_wind_start:i_wind_finish), s_wind_sample);
y_center_wind = interp1(s_wind, y_center(i_wind_start:i_wind_finish), s_wind_sample);
nl_wind = interp1(s_wind, nl(i_wind_start:i_wind_finish), s_wind_sample);
nr_wind = interp1(s_wind, nr(i_wind_start:i_wind_finish), s_wind_sample);
theta_wind = interp1(s_wind, theta(i_wind_start:i_wind_finish), s_wind_sample);

n_rows_wind = 5;
for i = 1 : numel(s_wind_sample)
    n_values = linspace(-nl_wind(i)+0.5,nr_wind(i)-0.5,n_rows_wind);
    for j = 1 : n_rows_wind
        draw_arrow([x_center_wind(i) + n_values(j)*sin(theta_wind(i)),x_center_wind(i) + n_values(j)*sin(theta_wind(i)) + 3*wind(1)], ...
            [y_center_wind(i) - n_values(j)*cos(theta_wind(i)),y_center_wind(i) - n_values(j)*cos(theta_wind(i)) + 3*wind(2)],0.2,4.0,[0,1,0])
    end
end


% (2) Plot throttle up to i
col = -simulation_data.("chassis.throttle")(1:i_frame)';  % This is the color, vary with x in this case.
col = min(max(-1,col),1);
max_col = max(col);
col(col>0) = col(col>0)/max_col;
cMap = interp1([0;0.5;1],[0 1 0; 1 1 0; 1 0 0],linspace(0,1,256));
z = zeros(size(x));
surface([x(max(1,i_frame-1400):i_frame),x(max(1,i_frame-1400):i_frame)],[-y(max(1,i_frame-1400):i_frame),-y(max(1,i_frame-1400):i_frame)],[z(max(1,i_frame-1400):i_frame),z(max(1,i_frame-1400):i_frame)],[col(max(1,i_frame-1400):i_frame)',col(max(1,i_frame-1400):i_frame)'],...s =
    'facecol','no',...
    'edgecol','interp',...
    'linew',4);
colormap(cMap)
ax.CLim = [-1,1];

% (n) Draw car
x_rr = simulation_data.("rear-axle.right-tire.position.x")(i_frame);
y_rr = simulation_data.("rear-axle.right-tire.position.y")(i_frame);
psi  = simulation_data.("chassis.attitude.yaw")(i_frame);

plot_f1(x_rr,-y_rr,rad2deg(psi)+180,1.6+1.8, 0.73*2);



% (3) Set camera
camera_width = 30.0;
xlim([simulation_data.x_center(i_frame)-1.777*camera_width*0.5,simulation_data.x_center(i_frame)+1.777*camera_width*0.5])
ylim([simulation_data.y_center(i_frame)-camera_width*0.5,simulation_data.y_center(i_frame)+camera_width*0.5])

% (n) Dashboard
ax_dashboard = axes('Position',[0.0 0.4 1.0 0.59]);

hold on

% (n.1) Throttle and brake
plot_throttle_brake(simulation_data.("chassis.throttle")(i_frame),simulation_data.("front-axle.steering-angle")(i_frame)*20,simulation_data.("chassis.brake-bias")(i_frame));

% (n.2) Acceleration
%plot_acceleration(simulation_data.("chassis.acceleration.x"),simulation_data.("chassis.acceleration.y"),i);

% (n.3) Tires
understeer_ind = rad2deg(simulation_data.("chassis.understeer_oversteer_indicator"));

Fz_max = -min([simulation_data.("chassis.Fz_fl");simulation_data.("chassis.Fz_fr");simulation_data.("chassis.Fz_rl");simulation_data.("chassis.Fz_rr")]);
plot_tires(simulation_data,i_frame,vehicle_data,Fz_max, understeer_ind);

% (n.4) Track map
%plot_track_map(x_center,y_center,x(i_frame),-y(i_frame));

% (n.5) Telemetry
%plot_telemetry(i,t,simulation_data)

ax_dashboard.XLim = [-3.3,  8.1];
ax_dashboard.YLim = [ax_dashboard.YLim(1)-ax_dashboard.YLim(2)+1.2,  1.2];
ax_dashboard.Visible = 'off';
end


function plot_throttle_brake(throttle_raw,delta_raw,brake_bias_raw)

% Trim throttle
throttle_raw = min(max(throttle_raw,-1),1);
patch([-0.2,-0.2,1.2,1.2,-0.2],[-0.1,1.1,1.1,-0.1,-0.1],[1,1,1])
plot([-0.2,-0.2,1.2,1.2,-0.2],[-0.1,1.1,1.1,-0.1,-0.1],'-k','LineWidth',3)

% Plot throttle and brake
throttle = throttle_raw; throttle(throttle < 0) = 0.0;
brake = -throttle_raw; brake(brake < 0) = 0.0;
plot([0,0,0.1,0.1,0],[0,1,1,0,0],'-k');

plot([0.15,0.15,0.25,0.25,0.15],[0,1,1,0,0],'-k');
patch([0,0,0.1,0.1,0],[0,brake,brake,0,0],[1,0,0]);
patch([0.15,0.15,0.25,0.25,0.15],[0,throttle,throttle,0,0],[0,1,0]);
plot([0,0.02],[0.25,0.25],'-k')
plot([0,0.02],[0.5,0.5],'-k')
plot([0,0.02],[0.75,0.75],'-k')
plot([0.08,0.1],[0.25,0.25],'-k')
plot([0.08,0.1],[0.5,0.5],'-k')
plot([0.08,0.1],[0.75,0.75],'-k')
plot(0.15+[0,0.02],[0.25,0.25],'-k')
plot(0.15+[0,0.02],[0.5,0.5],'-k')
plot(0.15+[0,0.02],[0.75,0.75],'-k')
plot(0.15+[0.08,0.1],[0.25,0.25],'-k')
plot(0.15+[0.08,0.1],[0.5,0.5],'-k')
plot(0.15+[0.08,0.1],[0.75,0.75],'-k')

% Plot steering wheel
theta = linspace(0,2*pi,100);
plot(0.75+0.25*cos(theta),0.5+0.25*sin(theta),'-k','LineWidth',2,'MarkerSize',20);


plot(0.75,0.5,'ok','MarkerFaceColor',[0,0,0]);
plot([0.75,0.75],[0.5+0.25,0.5+0.35],'-k','LineWidth',1);
plot([0.75+0.25*sin(delta_raw),0.75+0.35*sin(delta_raw)],[0.5+0.25*cos(delta_raw),0.5+0.35*cos(delta_raw)],'-k','LineWidth',1);
plot([0.75+0.3*sin(linspace(0,delta_raw,100))],[0.5+0.3*cos(linspace(0,delta_raw,100))],'Color','#D95319','LineWidth',2);

plot([0.75,0.75+0.25*sin(delta_raw+deg2rad(90))],[0.5,0.5+0.25*cos(delta_raw+deg2rad(90))],'-k','LineWidth',2);
plot([0.75,0.75+0.25*sin(delta_raw-deg2rad(180))],[0.5,0.5+0.25*cos(delta_raw-deg2rad(180))],'-k','LineWidth',2);
plot([0.75,0.75+0.25*sin(delta_raw+deg2rad(270))],[0.5,0.5+0.25*cos(delta_raw+deg2rad(270))],'-k','LineWidth',2);

% Plot brake bias
plot([0.35,1.10,1.10,0.35,0.35],[0.05,0.05,0.1,0.1,0.05],'-k');
bb_mapping = @(bb)( 0.35 + (1.10-0.35)*bb );
patch([bb_mapping(brake_bias_raw),bb_mapping(brake_bias_raw)+0.025,bb_mapping(brake_bias_raw)-0.025,bb_mapping(brake_bias_raw)],...
    [0.08,0.0,0.0,0.08],[0.05,0.05,0.05]);
text(1.10,-0.005,'F','FontWeight','bold','HorizontalAlignment', 'center');
text(0.35,-0.005,'R','FontWeight','bold','HorizontalAlignment', 'center');
text(0.5*(0.35+1.10),0.15,'brake-bias','FontName','Courier','FontWeight','bold','HorizontalAlignment', 'center');
axis equal


end

function plot_tires(simulation_data,i,vehicle_data,Fz_max, understeer_ind)

ref_load_1 = vehicle_data.front_tire.reference_load_1.Text;
ref_load_2 = vehicle_data.front_tire.reference_load_2.Text;
xlb = -3.2;
xub = -0.2;
xspan = xub-xlb;

wb = 2;
track = 0.4*2;

r_rr = [xlb+0.5*xspan+0.5*track,-1.8];
r_rl = r_rr - [track,0.0];
r_fr = r_rr + [0,wb];
r_fl = r_fr - [track,0.0];

patch([xlb,xlb,xub,xub,xlb],[-2.5,1.1,1.1,-2.5,-2.5],[1,1,1]);
plot([xlb,xlb,xub,xub,xlb],[-2.5,1.1,1.1,-2.5,-2.5],'-k','LineWidth',3);


% Plot f1
plot_f1(r_rr(1),r_rr(2),90,wb,track);

% (1) Get normal forces
Fz_fl = -simulation_data.("chassis.Fz_fl")(i)*vehicle_data.chassis.mass*9.81;
Fz_fr = -simulation_data.("chassis.Fz_fr")(i)*vehicle_data.chassis.mass*9.81;
Fz_rl = -simulation_data.("chassis.Fz_rl")(i)*vehicle_data.chassis.mass*9.81;
Fz_rr = -simulation_data.("chassis.Fz_rr")(i)*vehicle_data.chassis.mass*9.81;

% (4) Get maximum kappa and lambda
mu_x_max_fl = (Fz_fl - ref_load_1)*(vehicle_data.front_tire.mu_x_max_2  - vehicle_data.front_tire.mu_x_max_1 )/(ref_load_2 - ref_load_1) + vehicle_data.front_tire.mu_x_max_1;
mu_x_max_fr = (Fz_fr - ref_load_1)*(vehicle_data.front_tire.mu_x_max_2  - vehicle_data.front_tire.mu_x_max_1 )/(ref_load_2 - ref_load_1) + vehicle_data.front_tire.mu_x_max_1;
mu_x_max_rl = (Fz_rl - ref_load_1)*(vehicle_data.rear_tire.mu_x_max_2  - vehicle_data.rear_tire.mu_x_max_1 )/(ref_load_2 - ref_load_1) + vehicle_data.rear_tire.mu_x_max_1;
mu_x_max_rr = (Fz_rr - ref_load_1)*(vehicle_data.rear_tire.mu_x_max_2  - vehicle_data.rear_tire.mu_x_max_1 )/(ref_load_2 - ref_load_1) + vehicle_data.rear_tire.mu_x_max_1;

mu_y_max_fl = (Fz_fl - ref_load_1)*(vehicle_data.front_tire.mu_y_max_2  - vehicle_data.front_tire.mu_y_max_1 )/(ref_load_2 - ref_load_1) + vehicle_data.front_tire.mu_y_max_1;
mu_y_max_fr = (Fz_fr - ref_load_1)*(vehicle_data.front_tire.mu_y_max_2  - vehicle_data.front_tire.mu_y_max_1 )/(ref_load_2 - ref_load_1) + vehicle_data.front_tire.mu_y_max_1;
mu_y_max_rl = (Fz_rl - ref_load_1)*(vehicle_data.rear_tire.mu_y_max_2  - vehicle_data.rear_tire.mu_y_max_1 )/(ref_load_2 - ref_load_1) + vehicle_data.rear_tire.mu_y_max_1;
mu_y_max_rr = (Fz_rr - ref_load_1)*(vehicle_data.rear_tire.mu_y_max_2  - vehicle_data.rear_tire.mu_y_max_1 )/(ref_load_2 - ref_load_1) + vehicle_data.rear_tire.mu_y_max_1;

kappa_max_fl = (Fz_fl - ref_load_1)*(vehicle_data.front_tire.kappa_max_2  - vehicle_data.front_tire.kappa_max_1 )/(ref_load_2 - ref_load_1) + vehicle_data.front_tire.kappa_max_1;
kappa_max_fr = (Fz_fr - ref_load_1)*(vehicle_data.front_tire.kappa_max_2  - vehicle_data.front_tire.kappa_max_1 )/(ref_load_2 - ref_load_1) + vehicle_data.front_tire.kappa_max_1;
kappa_max_rl = (Fz_rl - ref_load_1)*(vehicle_data.rear_tire.kappa_max_2  - vehicle_data.rear_tire.kappa_max_1 )/(ref_load_2 - ref_load_1) + vehicle_data.rear_tire.kappa_max_1;
kappa_max_rr = (Fz_rr - ref_load_1)*(vehicle_data.rear_tire.kappa_max_2  - vehicle_data.rear_tire.kappa_max_1 )/(ref_load_2 - ref_load_1) + vehicle_data.rear_tire.kappa_max_1;

lambda_max_fl = deg2rad((Fz_fl - ref_load_1)*(vehicle_data.front_tire.lambda_max_2.Text  - vehicle_data.front_tire.lambda_max_1.Text )/(ref_load_2 - ref_load_1) + vehicle_data.front_tire.lambda_max_1.Text);
lambda_max_fr = deg2rad((Fz_fr - ref_load_1)*(vehicle_data.front_tire.lambda_max_2.Text  - vehicle_data.front_tire.lambda_max_1.Text )/(ref_load_2 - ref_load_1) + vehicle_data.front_tire.lambda_max_1.Text);
lambda_max_rl = deg2rad((Fz_rl - ref_load_1)*(vehicle_data.rear_tire.lambda_max_2.Text  - vehicle_data.rear_tire.lambda_max_1.Text )/(ref_load_2 - ref_load_1) + vehicle_data.rear_tire.lambda_max_1.Text);
lambda_max_rr = deg2rad((Fz_rr - ref_load_1)*(vehicle_data.rear_tire.lambda_max_2.Text  - vehicle_data.rear_tire.lambda_max_1.Text )/(ref_load_2 - ref_load_1) + vehicle_data.rear_tire.lambda_max_1.Text);

% (5) Get current kappa and lambda
kappa_fl = simulation_data.("front-axle.left-tire.true_kappa")(i);
kappa_fr = simulation_data.("front-axle.right-tire.true_kappa")(i);
kappa_rl = simulation_data.("rear-axle.left-tire.true_kappa")(i);
kappa_rr = simulation_data.("rear-axle.right-tire.true_kappa")(i);

lambda_fl = simulation_data.("front-axle.left-tire.lambda")(i);
lambda_fr = simulation_data.("front-axle.right-tire.lambda")(i);
lambda_rl = simulation_data.("rear-axle.left-tire.lambda")(i);
lambda_rr = simulation_data.("rear-axle.right-tire.lambda")(i);


% (6) Compute rho
rho_fl = sqrt((kappa_fl/kappa_max_fl)^2 + (lambda_fl/lambda_max_fl)^2);
rho_fr = sqrt((kappa_fr/kappa_max_fr)^2 + (lambda_fr/lambda_max_fr)^2);
rho_rl = sqrt((kappa_rl/kappa_max_rl)^2 + (lambda_rl/lambda_max_rl)^2);
rho_rr = sqrt((kappa_rr/kappa_max_rr)^2 + (lambda_rr/lambda_max_rr)^2);


scale = 1/8;
Fref = vehicle_data.chassis.mass*9.81;


% (8) Get tire forces
Fx_fl = simulation_data.("front-axle.left-tire.force.x")(i);
Fx_fr = simulation_data.("front-axle.right-tire.force.x")(i);
Fx_rl = simulation_data.("rear-axle.left-tire.force.x")(i);
Fx_rr = simulation_data.("rear-axle.right-tire.force.x")(i);

Fy_fl = simulation_data.("front-axle.left-tire.force.y")(i);
Fy_fr = simulation_data.("front-axle.right-tire.force.y")(i);
Fy_rl = simulation_data.("rear-axle.left-tire.force.y")(i);
Fy_rr = simulation_data.("rear-axle.right-tire.force.y")(i);

% (3) Draw maximum grip ellipses
delta = simulation_data.("front-axle.steering-angle")(i);

patch(r_fl(1) + scale*(-simulation_data.("chassis.Fz_fl")(i))*mu_y_max_fl*cos(linspace(0,2*pi,100)), r_fl(2)+scale*(-simulation_data.("chassis.Fz_fl")(i))*mu_x_max_fl*sin(linspace(0,2*pi,100)),[1,1,1]);
patch(r_fr(1) + scale*(-simulation_data.("chassis.Fz_fr")(i))*mu_y_max_fr*cos(linspace(0,2*pi,100)), r_fr(2)+scale*(-simulation_data.("chassis.Fz_fr")(i))*mu_x_max_fr*sin(linspace(0,2*pi,100)),[1,1,1]);
patch(r_rl(1) + scale*(-simulation_data.("chassis.Fz_rl")(i))*mu_y_max_rl*cos(linspace(0,2*pi,100)), r_rl(2)+scale*(-simulation_data.("chassis.Fz_rl")(i))*mu_x_max_rl*sin(linspace(0,2*pi,100)),[1,1,1]);
patch(r_rr(1) + scale*(-simulation_data.("chassis.Fz_rr")(i))*mu_y_max_rr*cos(linspace(0,2*pi,100)), r_rr(2)+scale*(-simulation_data.("chassis.Fz_rr")(i))*mu_x_max_rr*sin(linspace(0,2*pi,100)),[1,1,1]);

F_fl_rotated = Fx_fl*[cos(delta),sin(delta)] + Fy_fl*[-sin(delta),cos(delta)];
F_fr_rotated = Fx_fr*[cos(delta),sin(delta)] + Fy_fr*[-sin(delta),cos(delta)];

draw_arrow([r_fl(1),r_fl(1)+scale*F_fl_rotated(2)/Fref], [r_fl(2), r_fl(2) + scale*F_fl_rotated(1)/Fref]);
draw_arrow([r_fr(1),r_fr(1)+scale*F_fr_rotated(2)/Fref], [r_fr(2), r_fr(2) + scale*F_fr_rotated(1)/Fref]);
draw_arrow([r_rl(1),r_rl(1)+scale*Fy_rl/Fref], [r_rl(2), r_rl(2) + scale*Fx_rl/Fref]);
draw_arrow([r_rr(1),r_rr(1)+scale*Fy_rr/Fref], [r_rr(2), r_rr(2) + scale*Fx_rr/Fref]);

qa = [simulation_data.("chassis.Fz_fl")'; simulation_data.("chassis.Fz_fr")'; simulation_data.("chassis.Fz_rl")'; simulation_data.("chassis.Fz_rr")'];

plot([r_fl(1) -  1.5*scale*(-qa(1,i))*mu_y_max_fl*sin(delta), r_fl(1) +  1.5*scale*(-qa(1,i))*mu_y_max_fl*sin(delta)],...
    [r_fl(2) -  1.5*scale*(-qa(1,i))*mu_y_max_fl*cos(delta), r_fl(2) +  1.5*scale*(-qa(1,i))*mu_y_max_fl*cos(delta)],'-.k')

plot([r_fr(1) -  1.5*scale*(-qa(2,i))*mu_y_max_fr*sin(delta), r_fr(1) +  1.5*scale*(-qa(2,i))*mu_y_max_fr*sin(delta)],...
    [r_fr(2) -  1.5*scale*(-qa(2,i))*mu_y_max_fr*cos(delta), r_fr(2) +  1.5*scale*(-qa(2,i))*mu_y_max_fr*cos(delta)],'-.k')

% Draw slip measurement
opts = optimoptions('fminunc','Display','off');
max_rho = fminunc(@(x)(-sin(vehicle_data.front_tire.Qy*atan(x*pi/(2.0*atan(vehicle_data.front_tire.Qy))))),0,opts);

plot_slip_bar(max_rho,r_fl, qa(1,i), mu_y_max_fl, rho_fl, scale);
plot_slip_bar(max_rho,r_fr, qa(2,i), mu_y_max_fr, rho_fr, scale);
plot_slip_bar(max_rho,r_rl, qa(3,i), mu_y_max_rl, rho_rl, scale);
plot_slip_bar(max_rho,r_rr, qa(4,i), mu_y_max_rr, rho_rr, scale);

% Draw normal load bar
plot_normal_load_bar(r_fl+[scale*(-qa(1,i))*mu_y_max_fl,0], -qa(1,i), Fz_max)
plot_normal_load_bar(r_fr+[scale*(-qa(2,i))*mu_y_max_fr,0], -qa(2,i), Fz_max)
plot_normal_load_bar(r_rl+[scale*(-qa(3,i))*mu_y_max_rl,0], -qa(3,i), Fz_max)
plot_normal_load_bar(r_rr+[scale*(-qa(4,i))*mu_y_max_rr,0], -qa(4,i), Fz_max)

% Write tire properties

% Write the understeer/oversteer indicator

if ( abs(understeer_ind) < 0.1 )
    % Neutral steering
    patch([-1.1,-0.3,-0.3,-1.1,-1.1],[0.7,0.7,0.9,0.9,0.7],[0.9290, 0.6940, 0.1250],'LineStyle','none');
    text(-0.7,0.8,'Neutral','HorizontalAlignment','Center','FontName','Courier','FontWeight','bold');
elseif ( understeer_ind < 0.0 )
    % Under steering
    patch([-1.1,-0.3,-0.3,-1.1,-1.1],[0.7,0.7,0.9,0.9,0.7],[0, 0.4470, 0.7410],'LineStyle','none');
    text(-0.7,0.8,'Understeer','Color',[1,1,1],'HorizontalAlignment','Center','FontName','Courier','FontWeight','bold');
else
    % Over steering
    patch([-1.1,-0.3,-0.3,-1.1,-1.1],[0.7,0.7,0.9,0.9,0.7],[0.8500, 0.3250, 0.0980],'LineStyle','none');
    text(-0.7,0.8,'Oversteer','Color',[1,1,1],'HorizontalAlignment','Center','FontName','Courier','FontWeight','bold');
end

% Draw the velocity
velocity_scaling = 1/(100/3.6);
draw_arrow([xlb+0.5*xspan,xlb+0.5*xspan+simulation_data.("chassis.velocity.y")(i)*velocity_scaling],[-1.8+0.85,-1.8+.85+simulation_data.("chassis.velocity.x")(i)*velocity_scaling],0.1,4,[ 0    0.4470    0.7410]);
draw_arrow([xlb+0.5*xspan-simulation_data.("chassis.aerodynamics.aerodynamic_velocity.y")(i)*velocity_scaling,xlb+0.5*xspan-simulation_data.("chassis.aerodynamics.aerodynamic_velocity.y")(i)*velocity_scaling+simulation_data.("chassis.aerodynamics.wind_velocity_body.y")(i)*velocity_scaling],[-1.8+0.85-simulation_data.("chassis.aerodynamics.aerodynamic_velocity.x")(i)*velocity_scaling,-1.8+.85-simulation_data.("chassis.aerodynamics.aerodynamic_velocity.x")(i)*velocity_scaling+simulation_data.("chassis.aerodynamics.wind_velocity_body.x")(i)*velocity_scaling],0.3,4,[0,1,0])
draw_arrow([xlb+0.5*xspan,xlb+0.5*xspan-simulation_data.("chassis.aerodynamics.aerodynamic_velocity.y")(i)*velocity_scaling],[-1.8+0.85,-1.8+.85-simulation_data.("chassis.aerodynamics.aerodynamic_velocity.x")(i)*velocity_scaling],0.1,4,[1,0,0])

end

function draw_arrow(x,y,varargin)
if nargin > 2
    head_length = varargin{1};
    line_width = varargin{2};
    color = varargin{3};
else
    head_length = 0.4;
    line_width = 2;
    color = [0.01,0.01,0.01];
end

plot([x(1),x(1) + (1-head_length)*(x(2)-x(1))],[y(1),y(1) + (1-head_length)*(y(2)-y(1))],'Color',color,'LineWidth',line_width);


triangle_base = [x(1),y(1)] + (1-head_length)*[x(2)-x(1),y(2)-y(1)];
dr = [x(2)-x(1),y(2)-y(1)];

triangle_P0 = triangle_base + [dr(2),-dr(1)]*head_length*0.4;
triangle_P1 = triangle_base - [dr(2),-dr(1)]*head_length*0.4;

patch([x(2),triangle_P0(1),triangle_P1(1)], [y(2), triangle_P0(2), triangle_P1(2)],color);
end

function plot_slip_bar(max_rho,r_c, qa_i, mu_max, rho, scale)
patch([r_c(1) - 1.5*scale*(-qa_i)*mu_max*sind(linspace(30,180,50)), r_c(1) - 1.1*scale*(-qa_i)*mu_max*sind(linspace(180,30,50))],...
    [r_c(2) - 1.5*scale*(-qa_i)*mu_max*cosd(linspace(30,180,50)), r_c(2) - 1.1*scale*(-qa_i)*mu_max*cosd(linspace(180,30,50))],[1,1,1]);

patch([r_c(1) - 1.5*scale*(-qa_i)*mu_max*sind(linspace(180,215,50)), r_c(1) - 1.1*scale*(-qa_i)*mu_max*sind(linspace(215,180,50))],...
    [r_c(2) - 1.5*scale*(-qa_i)*mu_max*cosd(linspace(180,215,50)), r_c(2) - 1.1*scale*(-qa_i)*mu_max*cosd(linspace(215,180,50))],[1,1,1]);

theta_max = interp1([0,max_rho],[30,180],min(rho,max_rho));
x_surf = [r_c(1) - 1.5*scale*(-qa_i)*mu_max*sind(linspace(30,theta_max,50)); r_c(1) - 1.1*scale*(-qa_i)*mu_max*sind(linspace(30,theta_max,50))];
y_surf = [r_c(2) - 1.5*scale*(-qa_i)*mu_max*cosd(linspace(30,theta_max,50)); r_c(2) - 1.1*scale*(-qa_i)*mu_max*cosd(linspace(30,theta_max,50))];


col = (linspace(30,theta_max,50)-30)/(180-30);
surface(x_surf,y_surf,zeros(2,50),[6*col;6*col], 'facecol','interp',...
    'edgecol','no',...
    'linew',4);
% Plot the remaining if exceeds in #80461B
if ( rho > max_rho )
    theta_max = interp1([0,max_rho,210/180*max_rho],[0,180,210],min(rho,210/180*max_rho));
    x_surf = [r_c(1) - 1.5*scale*(-qa_i)*mu_max*sind(linspace(180,theta_max,50)), r_c(1) - 1.1*scale*(-qa_i)*mu_max*sind(linspace(theta_max,180,50))];
    y_surf = [r_c(2) - 1.5*scale*(-qa_i)*mu_max*cosd(linspace(180,theta_max,50)), r_c(2) - 1.1*scale*(-qa_i)*mu_max*cosd(linspace(theta_max,180,50))];
    patch(x_surf,y_surf,[139/255, 0, 0]);
end
end

function plot_normal_load_bar(r_c, Fz, Fz_max)
bar_len = 0.4;
patch([r_c(1)+0.01,r_c(1)+0.01,r_c(1)+0.08,r_c(1)+0.08],[r_c(2)-bar_len,r_c(2),r_c(2),r_c(2)-bar_len],[1,1,1])
plot([r_c(1)+0.01,r_c(1)+0.01,r_c(1)+0.08,r_c(1)+0.08,r_c(1)+0.01],[r_c(2)-bar_len,r_c(2),r_c(2),r_c(2),r_c(2)],'-k')

x_surf = [(r_c(1)+0.01)*ones(1,100);(r_c(1)+0.08)*ones(1,100)];
y_surf = [r_c(2)-bar_len + bar_len*linspace(0,Fz/Fz_max,100);r_c(2)-bar_len + bar_len*linspace(0,Fz/Fz_max,100)];
col = 6*[linspace(0,Fz/Fz_max,100);linspace(0,Fz/Fz_max,100)];

surface(x_surf,y_surf,zeros(2,100),col, 'facecol','interp',...
    'edgecol','no',...
    'linew',4);
end
