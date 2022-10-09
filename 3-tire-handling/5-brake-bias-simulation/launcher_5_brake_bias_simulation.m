if (~exist('restart','var'))
    restart = true;
end

set(groot,'defaultAxesFontName','Formula1');

if (restart)
    clear all
    clc
    close all
    
    prefix = '/Users/juanmanzanero/Documents/software/fastest-lap';
    lib_suffix = 'dylib';
    % We use 0.3.2 version: a version where the fitness function of LTS is the endpoint speed rather than time
    % Full list of changes:
    %   -> Added extra constraint to LTS: v = 0
    %   -> Changed q[ITIME] by q[IU] in fitness function
    %   -> Changed minimum speed to 20.0*KMH in LTS.
    fastest_lap_version = '0.3.2'; 
    
    addpath([prefix,'/src/main/matlab']);
    
    % (1) Load library
    if libisloaded('libfastestlapc')
        unloadlibrary libfastestlapc
    end
    
    loadlibrary([prefix,'/build/lib/libfastestlapc.',fastest_lap_version,'.',lib_suffix],[prefix,'/src/main/c/fastestlapc.h'],'alias','libfastestlapc');
    
    
    % (2) Construct track
    track = 'straight';
    calllib('libfastestlapc','create_track_from_xml',track,'straight.xml');
    n_points = 1000;
    s = linspace(0,1000,n_points+1);
    
    % (3) Construct car
    vehicle = 'car';
    calllib("libfastestlapc","create_vehicle_from_xml",vehicle,[prefix,'/database/vehicles/f1/limebeer-2014-f1.xml']);
    
    % (4) Get f1-3dof variable names
    [n_state, n_algebraic, n_control, n_output] = calllib('libfastestlapc','vehicle_type_get_sizes',0,0,0,0,'f1-3dof');
    n_char = 100;
    state_names = cell(1,n_state);               state_names(:)           = {blanks(n_char)};
    algebraic_state_names = cell(1,n_algebraic); algebraic_state_names(:) = {blanks(n_char)};
    control_names = cell(1,n_control);           control_names(:)         = {blanks(n_char)};
    output_names = cell(1,n_output);             output_names(:)          = {blanks(n_char)};
    [key_name, state_names, algebraic_state_names, control_names, output_names] = calllib('libfastestlapc','vehicle_type_get_names',blanks(n_char), state_names, algebraic_state_names, control_names, output_names, n_char, 'f1-3dof');
    
    % (4) Run steady-state computation at 250km/h
    [q0,qa0,u0] = calllib('libfastestlapc','steady_state',zeros(1,n_state), zeros(1,n_algebraic), zeros(1,n_control), vehicle, 320.0/3.6, 0.0, 0.0);
    
    % (4.1) Store it in the internal memory
    calllib('libfastestlapc','create_vector','trim_at_300kph/state',n_state,q0);
    calllib('libfastestlapc','create_vector','trim_at_300kph/algebraic_state',n_algebraic,qa0);
    calllib('libfastestlapc','create_vector','trim_at_300kph/control',n_control,u0);
    
    
    % (6) Run optimal acceleration with brake-bias at 60%    
    
    % (6.1) Write options
    options =          '<options>';
    options = [options,    '<closed_simulation> false </closed_simulation>'];
    options = [options,    '<initial_condition>'];
    options = [options,    '    <q  from_table="trim_at_300kph/state"/>'];
    options = [options,    '    <qa from_table="trim_at_300kph/algebraic_state"/>'];
    options = [options,    '    <u  from_table="trim_at_300kph/control"/>'];
    options = [options,    '</initial_condition>'];
    options = [options,    '<control_variables>'];
    options = [options,    '    <chassis.throttle optimal_control_type="full-mesh">'];
    options = [options,    '        <dissipation> 1.0e-2 </dissipation>'];
    options = [options,    '    </chassis.throttle>'];
%    options = [options,    '    <chassis.brake-bias optimal_control_type="constant">'];
%    options = [options,    '    </chassis.brake-bias>'];
    options = [options,    '</control_variables>'];
    options = [options,    '<output_variables>'];
    options = [options,    '    <prefix>run/</prefix>'];
    options = [options,    '</output_variables>'];
    options = [options,    '<sigma> 0.5 </sigma>'];
    options = [options,'</options>'];
    
    % (6.2) Run
    n_points = 100;
    s = linspace(0,n_points,n_points+1);
    
    fprintf('[INFO] launcher_3_full_throttle_simulation -> [start] optimal laptime simulation\n');
    calllib("libfastestlapc","optimal_laptime",vehicle,track,n_points+1,s,options);
    fprintf('[INFO] launcher_3_full_throttle_simulation -> [end] optimal laptime simulation\n');
    
    % (6.3) Download
    run_60p = download_vectors('run/', {state_names{:}, algebraic_state_names{:}, control_names{:}, output_names{:}}, n_points+1);
    calllib('libfastestlapc','delete_variable','run/*');
    
    % (7) Run optimal acceleration with brake-bias at 63%    
    calllib('libfastestlapc','vehicle_set_parameter',vehicle,'vehicle/chassis/brake_bias',0.63);
    % (7.1) Write options
    options =          '<options>';
    options = [options,    '<closed_simulation> false </closed_simulation>'];
    options = [options,    '<initial_condition>'];
    options = [options,    '    <q  from_table="trim_at_300kph/state"/>'];
    options = [options,    '    <qa from_table="trim_at_300kph/algebraic_state"/>'];
    options = [options,    '    <u  from_table="trim_at_300kph/control"/>'];
    options = [options,    '</initial_condition>'];
    options = [options,    '<control_variables>'];
    options = [options,    '    <chassis.throttle optimal_control_type="full-mesh">'];
    options = [options,    '        <dissipation> 1.0e-2 </dissipation>'];
    options = [options,    '    </chassis.throttle>'];
%    options = [options,    '    <chassis.brake-bias optimal_control_type="constant">'];
%    options = [options,    '    </chassis.brake-bias>'];
    options = [options,    '</control_variables>'];
    options = [options,    '<output_variables>'];
    options = [options,    '    <prefix>run/</prefix>'];
    options = [options,    '</output_variables>'];
    options = [options,    '<sigma> 0.5 </sigma>'];
    options = [options,'</options>'];
    
    % (7.2) Run
    n_points = 100;
    s = linspace(0,n_points,n_points+1);
    
    fprintf('[INFO] launcher_3_full_throttle_simulation -> [start] optimal laptime simulation\n');
    calllib("libfastestlapc","optimal_laptime",vehicle,track,n_points+1,s,options);
    fprintf('[INFO] launcher_3_full_throttle_simulation -> [end] optimal laptime simulation\n');
    
    % (7.3) Download
    run_63p = download_vectors('run/', {state_names{:}, algebraic_state_names{:}, control_names{:}, output_names{:}}, n_points+1);
    calllib('libfastestlapc','delete_variable','run/*');    
    
    % (7) Run optimal acceleration with brake-bias at 63%    
    calllib('libfastestlapc','vehicle_set_parameter',vehicle,'vehicle/chassis/brake_bias',0.57);
    % (7.1) Write options
    options =          '<options>';
    options = [options,    '<closed_simulation> false </closed_simulation>'];
    options = [options,    '<initial_condition>'];
    options = [options,    '    <q  from_table="trim_at_300kph/state"/>'];
    options = [options,    '    <qa from_table="trim_at_300kph/algebraic_state"/>'];
    options = [options,    '    <u  from_table="trim_at_300kph/control"/>'];
    options = [options,    '</initial_condition>'];
    options = [options,    '<control_variables>'];
    options = [options,    '    <chassis.throttle optimal_control_type="full-mesh">'];
    options = [options,    '        <dissipation> 1.0e-2 </dissipation>'];
    options = [options,    '    </chassis.throttle>'];
%    options = [options,    '    <chassis.brake-bias optimal_control_type="constant">'];
%    options = [options,    '    </chassis.brake-bias>'];
    options = [options,    '</control_variables>'];
    options = [options,    '<output_variables>'];
    options = [options,    '    <prefix>run/</prefix>'];
    options = [options,    '</output_variables>'];
    options = [options,    '<sigma> 0.5 </sigma>'];
    options = [options,'</options>'];
    
    % (7.2) Run
    n_points = 100;
    s = linspace(0,n_points,n_points+1);
    
    fprintf('[INFO] launcher_3_full_throttle_simulation -> [start] optimal laptime simulation\n');
    calllib("libfastestlapc","optimal_laptime",vehicle,track,n_points+1,s,options);
    fprintf('[INFO] launcher_3_full_throttle_simulation -> [end] optimal laptime simulation\n');
    
    % (7.3) Download
    run_57p = download_vectors('run/', {state_names{:}, algebraic_state_names{:}, control_names{:}, output_names{:}}, n_points+1);
    calllib('libfastestlapc','delete_variable','run/*');    
        
    
    
    
    
    save('run','run_60p','run_63p','run_57p','s','n_points','vehicle','prefix','track','lib_suffix','fastest_lap_version','state_names','algebraic_state_names','control_names','output_names');
    clear all
    load('run');
    restart = false;
else
    clear all
    clc
    close all
    restart = false;
    load('run');
    
    % Load library
    if libisloaded('libfastestlapc')
        unloadlibrary libfastestlapc
    end
    
    loadlibrary([prefix,'/build/lib/libfastestlapc.',fastest_lap_version,'.',lib_suffix],[prefix,'/src/main/c/fastestlapc.h'],'alias','libfastestlapc');
    track = 'straight';
    calllib('libfastestlapc','create_track_from_xml',track,'straight.xml');
    calllib("libfastestlapc","create_vehicle_from_xml",vehicle,[prefix,'/database/vehicles/f1/limebeer-2014-f1.xml']);
    calllib('libfastestlapc','vehicle_change_track',vehicle,track);
    
    [n_state, n_algebraic, n_control, n_output] = calllib('libfastestlapc','vehicle_type_get_sizes',0,0,0,0,'f1-3dof');
    n_char = 100;
    state_names = cell(1,n_state);               state_names(:)           = {blanks(n_char)};
    algebraic_state_names = cell(1,n_algebraic); algebraic_state_names(:) = {blanks(n_char)};
    control_names = cell(1,n_control);           control_names(:)         = {blanks(n_char)};
    output_names = cell(1,n_output);             output_names(:)          = {blanks(n_char)};
    [key_name, state_names, algebraic_state_names, control_names, output_names] = calllib('libfastestlapc','vehicle_type_get_names',blanks(n_char), state_names, algebraic_state_names, control_names, output_names, n_char, 'f1-3dof');
    
end

%
% (1) Plot the simulation with brake-bias at 60%
%plot_simulation_1car(run_60p,s(1:n_points+1),vehicle,state_names,algebraic_state_names,control_names,output_names,'1_brake_bias_60p')
plot_simulation_1car_with_ghost(run_57p,s(1:n_points+1),vehicle,state_names,algebraic_state_names,control_names,output_names,'3_brake_bias_57p',run_60p)

%
% (2) Plot the two cars: only front brake and only rear brake
%n_points_100m = 160;
%s_only_one_axle = linspace(0,n_points_100m,n_points_100m+1);
%plot_comparison(run_only_front_brakes,run_only_rear_brakes,s_only_one_axle,vehicle,state_names,algebraic_state_names,control_names,output_names,'2_front_vs_rear_brake_turquoise')

%
%       Helper functions -----------------------------------------:-
%

function struct_out = download_vectors(prefix,names,n_elements)
struct_out = struct();

for i = 1 : numel(names)
    variable = calllib('libfastestlapc','download_vector',zeros(1,n_elements),n_elements,[prefix,names{i}]) ;
    
    variable_names_split = split(strrep(names{i},'-','_'),'.');
    
    struct_out = append_new_field(struct_out, variable, variable_names_split);
    
end

end

function run_out = transform_run(run_in,state_names,algebraic_names,control_names)
run_out = struct();

for i = 1 : numel(state_names)
    variable_names_split = split(strrep(state_names{i},'-','_'),'.');
    run_out = append_new_field(run_out, run_in.q(i,:), variable_names_split);
end

for i = 1 : numel(algebraic_names)
    variable_names_split = split(strrep(algebraic_names{i},'-','_'),'.');
    run_out = append_new_field(run_out, run_in.qa(i,:), variable_names_split);
end

for i = 1 : numel(control_names)
    variable_names_split = split(strrep(control_names{i},'-','_'),'.');
    run_out = append_new_field(run_out, run_in.u(i,:), variable_names_split);
end
end

function str_out = append_new_field(str_in, value, names)
if numel(names) == 1
    str_out = str_in;
    str_out.(names{1}) = value;
else
    if ~isfield(str_in,names{1})
        str_in.(names{1}) = [];
    end
    str_out = str_in;
    str_out.(names{1}) = append_new_field(str_in.(names{1}),value,names(2:end));
end
end

function plot_simulation_1car(run,s,vehicle,state_names,algebraic_state_names,control_names,output_names,file_name)

mkdir(['figs/',file_name]);
background_color =  [13/255, 17/255, 23/255];
text_color = [201,209,217]/255;
blue   = [0   0.447000000000000   0.741];
FPS = 60;
time = 0:1/FPS:run.time(end);
tire_data = readstruct('../limebeer-2014-f1.xml');

for i_frame = 1:numel(time)
    fprintf('frame: %d/%d\n',i_frame,length(time));
    
    q = [run.front_axle.left_tire.kappa; run.front_axle.right_tire.kappa; run.rear_axle.left_tire.kappa; run.rear_axle.right_tire.kappa;...
        run.chassis.velocity.x; run.chassis.velocity.y; run.chassis.omega.z; run.time; run.road.lateral_displacement; run.road.track_heading_angle];
    qa = [run.chassis.Fz_fl; run.chassis.Fz_fr; run.chassis.Fz_rl; run.chassis.Fz_rr];
    u = [run.front_axle.steering_angle; run.rear_axle.boost; run.chassis.throttle; run.chassis.brake_bias];
    
    q_i = interp1(run.time,q',time(i_frame));
    qa_i = interp1(run.time,qa',time(i_frame));
    u_i = interp1(run.time,u',time(i_frame));
    s_i = interp1(run.time,s,time(i_frame));
    
    screen_size = get(0,'ScreenSize');
    h = figure('Visible','off','Position',[0 0 screen_size(3) 1080.0/1920.0*screen_size(3)]);
    hold on
    
    % (1) Plot track
    kerb_color_length = 2.5;
    for i = -40:400
        patch([kerb_color_length*2*i,kerb_color_length*(2*i+1),kerb_color_length*(2*i+1),kerb_color_length*2*i],[4,4,4.5,4.5],[1,0,0]) ;
        patch([kerb_color_length*2*i,kerb_color_length*(2*i+1),kerb_color_length*(2*i+1),kerb_color_length*2*i],-[4,4,4.5,4.5],[1,0,0]) ;
    end
    for i = -40:400
        patch([kerb_color_length*(2*i+1), kerb_color_length*(2*i+2),kerb_color_length*(2*i+2),kerb_color_length*(2*i+1)],[4,4,4.5,4.5],[1,1,1]) ;
        patch([kerb_color_length*(2*i+1), kerb_color_length*(2*i+2),kerb_color_length*(2*i+2),kerb_color_length*(2*i+1)],-[4,4,4.5,4.5],[1,1,1]) ;
    end
    
    patch([-100,1100,1100,-100],[-4,-4,4,4],[129/255, 149/255, 179/255]);
    
    
    
    axis equal
    h.Color = [13/255, 17/255, 23/255];
    h.CurrentAxes.Visible = 'off';
    ax = gca;
    ax.Position = [0,0,1,1];
    
    % (n) Draw car
    x_rr = calllib("libfastestlapc","vehicle_get_output",vehicle,q_i,qa_i,u_i,s_i,'rear_axle.right_tire.x');
    y_rr = calllib("libfastestlapc","vehicle_get_output",vehicle,q_i,qa_i,u_i,s_i,'rear_axle.right_tire.y');
    a_x = calllib("libfastestlapc","vehicle_get_output",vehicle,q_i,qa_i,u_i,s_i,'ax');
    psi = q_i(end);
    
    plot_f1(x_rr,-y_rr,rad2deg(psi)+180,1.6+1.8, 0.73*2,'redbull');
    
    if ( abs(q_i(1) + 1.0) < 0.1 )
        
            text(x_rr + 1.6+ 1.8, 1.2,'Locked','FontName','Formula1','HorizontalAlignment','center','backgroundColor',[1,0,0],'FontSize',20)
        
    end
    
    if ( abs(q_i(3) + 1.0) < 0.1 )
        
            text(x_rr, 1.2,'Locked','FontName','Formula1','HorizontalAlignment','center','backgroundColor',[1,0,0],'FontSize',20)
        
    end
    
    % (3) Set camera
    camera_width = 20.0;
    xlim([s_i-1.777*camera_width*0.5,s_i+1.777*camera_width*0.5])
    ylim([-5,-5+camera_width])
    
    % (4) Plot the pacejka model and the point
    ax_pacejka = axes('Position',[0.1 0.55 1080.0/1920.0*0.4 0.4]);
    hold on
    kappa_plot = linspace(-0.3,0.3,1000);
    plot(kappa_plot,pacejka_model_longitudinal(-qa_i(1)*660*9.81,kappa_plot,tire_data),'LineWidth',2,'Color',text_color);
    plot(q_i(1),pacejka_model_longitudinal(-qa_i(1)*660*9.81,q_i(1),tire_data),'o','MarkerSize',25,'Color',[1,0,0],'LineWidth',4);
    plot(q_i(1),pacejka_model_longitudinal(-qa_i(1)*660*9.81,q_i(1),tire_data),'.','MarkerSize',25,'Color',[1,0,0],'LineWidth',4);
    
    if ( abs(q_i(1) + 1.0) < 0.1 )
        
            text(-0.8, 2000.0,'Locked','FontName','Formula1','HorizontalAlignment','center','backgroundColor',[1,0,0],'FontSize',30)
        
    end
    
    ax_pacejka.Color = background_color;
    ax_pacejka.XAxis.Color = text_color;
    ax_pacejka.YAxis.Color = text_color;
    ax_pacejka.XMinorTick = 'on';
    ax_pacejka.YMinorTick = 'on';
    ax_pacejka.LineWidth = 2;
    ax_pacejka.YAxis.Exponent = 0;
    xlabel('front tire longitudinal slip, kappa [-]','FontSize',20);
    annotation(h,'textbox',[0.075 0.95 1080.0/1920.0*0.5 0.05],'string','front tire longitudinal force, Fx [N]','FontSize',18,'FontName','Formula1','horizontalalignment','center','verticalAlignment','middle','Color',text_color,'LineStyle','none');
    box on
    grid on
    ax_pacejka.XMinorGrid = 'on';
    set(gca,'MinorGridLineStyle','-')
    ylim([-12000,12000]);
    xlim([-0.3,0.3]);
    ax_pacejka.FontSize = 16;
    
    % (4) Plot the pacejka model and the point
    ax_pacejka = axes('Position',[0.4 0.55 1080.0/1920.0*0.4 0.4]);
    hold on
    kappa_plot = linspace(-0.3,0.3,1000);
    plot(kappa_plot,pacejka_model_longitudinal(-qa_i(3)*660*9.81,kappa_plot,tire_data),'LineWidth',2,'Color',text_color);
    plot(q_i(3),pacejka_model_longitudinal(-qa_i(3)*660*9.81,q_i(3),tire_data),'o','MarkerSize',25,'Color',[1,0,0],'LineWidth',4);
    plot(q_i(3),pacejka_model_longitudinal(-qa_i(3)*660*9.81,q_i(3),tire_data),'.','MarkerSize',25,'Color',[1,0,0],'LineWidth',4);
    
    if ( abs(q_i(3) + 1.0) < 0.1 )
       
            text(-0.8, 2000.0,'Locked','FontName','Formula1','HorizontalAlignment','center','backgroundColor',[1,0,0],'FontSize',30)
        
    end
    
    ax_pacejka.Color = background_color;
    ax_pacejka.XAxis.Color = text_color;
    ax_pacejka.YAxis.Color = text_color;
    
    ax_pacejka.XMinorTick = 'on';
    ax_pacejka.YMinorTick = 'on';
    ax_pacejka.LineWidth = 2;
    ax_pacejka.YAxis.Exponent = 0;
    xlabel('rear tire longitudinal slip, kappa [-]','FontSize',20);
    
    annotation(h,'textbox',[0.375 0.95 1080.0/1920.0*0.5 0.05],'string','rear tire longitudinal force, Fx [N]','FontSize',18,'FontName','Formula1','horizontalalignment','center','verticalAlignment','middle','Color',text_color,'LineStyle','none');
    box on
    grid on
    ax_pacejka.XMinorGrid = 'on';
    set(gca,'MinorGridLineStyle','-')
    ylim([-12000,12000]);
    xlim([-0.3,0.3]);
    ax_pacejka.FontSize = 16;
    
    % (5) Plot throttle
    ax_throttle = axes('Position',[0.675 0.55 1080.0/1920.0*0.4*0.1 0.4]);
    patch([0,0,1,1,0],[0,-u_i(3)*100,-u_i(3)*100,0,0],blue);
    ylabel('Brake [%]')
    ax_throttle.Color = background_color;
    ax_throttle.XAxis.Color = text_color;
    ax_throttle.YAxis.Color = text_color;
    ax_throttle.FontSize = 14;
    ax_throttle.XMinorTick = 'on';
    ax_throttle.YMinorTick = 'on';
    ax_throttle.LineWidth = 2;
    ax_throttle.XTick = [];
    ylim([0,100]);
    box on
    
    % Plot velocity in the form of gauges
    ax_velocity = axes('Position',[0.7 0.50 1080.0/1920.0*0.5 0.5]);
    hold on;
    ax_velocity.Visible = 'off';
    patch([ - 1.5*sind(linspace(45,315,50)), -1.2*sind(linspace(315,45,50))],...
        [-1.5*cosd(linspace(45,315,50)), -1.2*cosd(linspace(315,45,50))],text_color);
    
    vel = max(0.0,q_i(5));
    angle_for_velocity = 45 + (315-45)*3.6*vel/350;
    patch([ - 1.5*sind(linspace(45,angle_for_velocity,50)), -1.2*sind(linspace(angle_for_velocity,45,50))],...
        [-1.5*cosd(linspace(45,angle_for_velocity,50)), -1.2*cosd(linspace(angle_for_velocity,45,50))],blue);
    
    % Put marks at several velocities
    velocities = 0:25:350;
    
    for velocity = velocities
        angle = 45 + (315-45)*velocity/velocities(end);
        text(-1.75*sind(angle), -1.75*cosd(angle),[num2str(velocity)],'horizontalAlignment','center','FontName','Formula1','Color',text_color,'FontSize',14)
    end
    text(0.0,2.0,'Velocity','FontSize',22,'FontName','Formula1','Color',text_color,'horizontalAlignment','center');
    text(0.0,0.0,[num2str(a_x/9.81,'%.1f'),'g'],'FontSize',32,'FontName','Formula1','Color',text_color,'horizontalAlignment','center');
    axis equal
    xlim([-2.0,2.0]);
    ylim([-1.5,2.5]);
    
    
    set(h, 'InvertHardcopy', 'off');
    print(h,['figs/',file_name,'/fig_',num2str(i_frame),'.png'],'-dpng','-r300');
    close(h)
end
end

function plot_simulation_1car_with_ghost(run,s,vehicle,state_names,algebraic_state_names,control_names,output_names,file_name,run_ghost)

mkdir(['figs/',file_name]);
background_color =  [13/255, 17/255, 23/255];
text_color = [201,209,217]/255;
blue   = [0   0.447000000000000   0.741];
FPS = 60;
time = 0:1/FPS:run.time(end);
tire_data = readstruct('../limebeer-2014-f1.xml');

for i_frame = 1:numel(time)
    fprintf('frame: %d/%d\n',i_frame,length(time));
    
    q = [run.front_axle.left_tire.kappa; run.front_axle.right_tire.kappa; run.rear_axle.left_tire.kappa; run.rear_axle.right_tire.kappa;...
        run.chassis.velocity.x; run.chassis.velocity.y; run.chassis.omega.z; run.time; run.road.lateral_displacement; run.road.track_heading_angle];
    qa = [run.chassis.Fz_fl; run.chassis.Fz_fr; run.chassis.Fz_rl; run.chassis.Fz_rr];
    u = [run.front_axle.steering_angle; run.rear_axle.boost; run.chassis.throttle; run.chassis.brake_bias];
    
    q_i = interp1(run.time,q',time(i_frame));
    qa_i = interp1(run.time,qa',time(i_frame));
    u_i = interp1(run.time,u',time(i_frame));
    s_i = interp1(run.time,s,time(i_frame));
    
    
    q_ghost = [run_ghost.front_axle.left_tire.kappa; run_ghost.front_axle.right_tire.kappa; run_ghost.rear_axle.left_tire.kappa; run_ghost.rear_axle.right_tire.kappa;...
        run_ghost.chassis.velocity.x; run_ghost.chassis.velocity.y; run_ghost.chassis.omega.z; run_ghost.time; run_ghost.road.lateral_displacement; run_ghost.road.track_heading_angle];
    qa_ghost = [run_ghost.chassis.Fz_fl; run_ghost.chassis.Fz_fr; run_ghost.chassis.Fz_rl; run_ghost.chassis.Fz_rr];
    u_ghost = [run_ghost.front_axle.steering_angle; run_ghost.rear_axle.boost; run_ghost.chassis.throttle; run_ghost.chassis.brake_bias];
    
    q_ghost_i = interp1(run_ghost.time,q_ghost',time(i_frame));
    qa_ghost_i = interp1(run_ghost.time,qa_ghost',time(i_frame));
    u_ghost_i = interp1(run_ghost.time,u_ghost',time(i_frame));
    s_ghost_i = interp1(run_ghost.time,s,time(i_frame));    
    
    screen_size = get(0,'ScreenSize');
    h = figure('Visible','off','Position',[0 0 screen_size(3) 1080.0/1920.0*screen_size(3)]);
    hold on
    
    % (1) Plot track
    kerb_color_length = 2.5;
    for i = -40:400
        patch([kerb_color_length*2*i,kerb_color_length*(2*i+1),kerb_color_length*(2*i+1),kerb_color_length*2*i],[4,4,4.5,4.5],[1,0,0]) ;
        patch([kerb_color_length*2*i,kerb_color_length*(2*i+1),kerb_color_length*(2*i+1),kerb_color_length*2*i],-[4,4,4.5,4.5],[1,0,0]) ;
    end
    for i = -40:400
        patch([kerb_color_length*(2*i+1), kerb_color_length*(2*i+2),kerb_color_length*(2*i+2),kerb_color_length*(2*i+1)],[4,4,4.5,4.5],[1,1,1]) ;
        patch([kerb_color_length*(2*i+1), kerb_color_length*(2*i+2),kerb_color_length*(2*i+2),kerb_color_length*(2*i+1)],-[4,4,4.5,4.5],[1,1,1]) ;
    end
    
    patch([-100,1100,1100,-100],[-4,-4,4,4],[129/255, 149/255, 179/255]);
    
    
    
    axis equal
    h.Color = [13/255, 17/255, 23/255];
    h.CurrentAxes.Visible = 'off';
    ax = gca;
    ax.Position = [0,0,1,1];
    
    % (n) Draw car
    x_rr = calllib("libfastestlapc","vehicle_get_output",vehicle,q_i,qa_i,u_i,s_i,'rear_axle.right_tire.x');
    y_rr = calllib("libfastestlapc","vehicle_get_output",vehicle,q_i,qa_i,u_i,s_i,'rear_axle.right_tire.y');
    a_x = calllib("libfastestlapc","vehicle_get_output",vehicle,q_i,qa_i,u_i,s_i,'ax');
    psi = q_i(end);
    
    x_ghost_rr = calllib("libfastestlapc","vehicle_get_output",vehicle,q_ghost_i,qa_ghost_i,u_ghost_i,s_ghost_i,'rear_axle.right_tire.x');
    y_ghost_rr = calllib("libfastestlapc","vehicle_get_output",vehicle,q_ghost_i,qa_ghost_i,u_ghost_i,s_ghost_i,'rear_axle.right_tire.y');
    psi_ghost = q_ghost_i(end);
    
    plot_f1(x_rr,-y_rr,rad2deg(psi)+180,1.6+1.8, 0.73*2,'redbull');
    plot_f1(x_ghost_rr,-y_ghost_rr,rad2deg(psi_ghost)+180,1.6+1.8, 0.73*2,'redbull',[1,1,1],0.2);
    
    if ( abs(q_i(1) + 1.0) < 0.1 )
        
            text(x_rr + 1.6+ 1.8, 1.2,'Locked','FontName','Formula1','HorizontalAlignment','center','backgroundColor',[1,0,0],'FontSize',20)
        
    end
    
    if ( abs(q_i(3) + 1.0) < 0.1 )
        
            text(x_rr, 1.2,'Locked','FontName','Formula1','HorizontalAlignment','center','backgroundColor',[1,0,0],'FontSize',20)
        
    end
    
    % (3) Set camera
    camera_width = 20.0;
    xlim([s_i-1.777*camera_width*0.5,s_i+1.777*camera_width*0.5])
    ylim([-5,-5+camera_width])
    
    % (4) Plot the pacejka model and the point
    ax_pacejka = axes('Position',[0.1 0.55 1080.0/1920.0*0.4 0.4]);
    hold on
    kappa_plot = linspace(-0.3,0.3,1000);
    plot(kappa_plot,pacejka_model_longitudinal(-qa_i(1)*660*9.81,kappa_plot,tire_data),'LineWidth',2,'Color',text_color);
    plot(q_i(1),pacejka_model_longitudinal(-qa_i(1)*660*9.81,q_i(1),tire_data),'o','MarkerSize',25,'Color',[1,0,0],'LineWidth',4);
    plot(q_i(1),pacejka_model_longitudinal(-qa_i(1)*660*9.81,q_i(1),tire_data),'.','MarkerSize',25,'Color',[1,0,0],'LineWidth',4);
    
    if ( abs(q_i(1) + 1.0) < 0.1 )
        
            text(-0.8, 2000.0,'Locked','FontName','Formula1','HorizontalAlignment','center','backgroundColor',[1,0,0],'FontSize',30)
        
    end
    
    ax_pacejka.Color = background_color;
    ax_pacejka.XAxis.Color = text_color;
    ax_pacejka.YAxis.Color = text_color;
    ax_pacejka.XMinorTick = 'on';
    ax_pacejka.YMinorTick = 'on';
    ax_pacejka.LineWidth = 2;
    ax_pacejka.YAxis.Exponent = 0;
    xlabel('front tire longitudinal slip, kappa [-]','FontSize',20);
    annotation(h,'textbox',[0.075 0.95 1080.0/1920.0*0.5 0.05],'string','front tire longitudinal force, Fx [N]','FontSize',18,'FontName','Formula1','horizontalalignment','center','verticalAlignment','middle','Color',text_color,'LineStyle','none');
    box on
    grid on
    ax_pacejka.XMinorGrid = 'on';
    set(gca,'MinorGridLineStyle','-')
    ylim([-12000,12000]);
    xlim([-0.3,0.3]);
    ax_pacejka.FontSize = 16;
    
    % (4) Plot the pacejka model and the point
    ax_pacejka = axes('Position',[0.4 0.55 1080.0/1920.0*0.4 0.4]);
    hold on
    kappa_plot = linspace(-0.3,0.3,1000);
    plot(kappa_plot,pacejka_model_longitudinal(-qa_i(3)*660*9.81,kappa_plot,tire_data),'LineWidth',2,'Color',text_color);
    plot(q_i(3),pacejka_model_longitudinal(-qa_i(3)*660*9.81,q_i(3),tire_data),'o','MarkerSize',25,'Color',[1,0,0],'LineWidth',4);
    plot(q_i(3),pacejka_model_longitudinal(-qa_i(3)*660*9.81,q_i(3),tire_data),'.','MarkerSize',25,'Color',[1,0,0],'LineWidth',4);
    
    if ( abs(q_i(3) + 1.0) < 0.1 )
       
            text(-0.8, 2000.0,'Locked','FontName','Formula1','HorizontalAlignment','center','backgroundColor',[1,0,0],'FontSize',30)
        
    end
    
    ax_pacejka.Color = background_color;
    ax_pacejka.XAxis.Color = text_color;
    ax_pacejka.YAxis.Color = text_color;
    
    ax_pacejka.XMinorTick = 'on';
    ax_pacejka.YMinorTick = 'on';
    ax_pacejka.LineWidth = 2;
    ax_pacejka.YAxis.Exponent = 0;
    xlabel('rear tire longitudinal slip, kappa [-]','FontSize',20);
    
    annotation(h,'textbox',[0.375 0.95 1080.0/1920.0*0.5 0.05],'string','rear tire longitudinal force, Fx [N]','FontSize',18,'FontName','Formula1','horizontalalignment','center','verticalAlignment','middle','Color',text_color,'LineStyle','none');
    box on
    grid on
    ax_pacejka.XMinorGrid = 'on';
    set(gca,'MinorGridLineStyle','-')
    ylim([-12000,12000]);
    xlim([-0.3,0.3]);
    ax_pacejka.FontSize = 16;
    
    % (5) Plot throttle
    ax_throttle = axes('Position',[0.675 0.55 1080.0/1920.0*0.4*0.1 0.4]);
    patch([0,0,1,1,0],[0,-u_i(3)*100,-u_i(3)*100,0,0],blue);
    ylabel('Brake [%]')
    ax_throttle.Color = background_color;
    ax_throttle.XAxis.Color = text_color;
    ax_throttle.YAxis.Color = text_color;
    ax_throttle.FontSize = 14;
    ax_throttle.XMinorTick = 'on';
    ax_throttle.YMinorTick = 'on';
    ax_throttle.LineWidth = 2;
    ax_throttle.XTick = [];
    ylim([0,100]);
    box on
    
    % Plot velocity in the form of gauges
    ax_velocity = axes('Position',[0.7 0.50 1080.0/1920.0*0.5 0.5]);
    hold on;
    ax_velocity.Visible = 'off';
    patch([ - 1.5*sind(linspace(45,315,50)), -1.2*sind(linspace(315,45,50))],...
        [-1.5*cosd(linspace(45,315,50)), -1.2*cosd(linspace(315,45,50))],text_color);
    
    vel = max(0.0,q_i(5));
    angle_for_velocity = 45 + (315-45)*3.6*vel/350;
    patch([ - 1.5*sind(linspace(45,angle_for_velocity,50)), -1.2*sind(linspace(angle_for_velocity,45,50))],...
        [-1.5*cosd(linspace(45,angle_for_velocity,50)), -1.2*cosd(linspace(angle_for_velocity,45,50))],blue);
    
    % Put marks at several velocities
    velocities = 0:25:350;
    
    for velocity = velocities
        angle = 45 + (315-45)*velocity/velocities(end);
        text(-1.75*sind(angle), -1.75*cosd(angle),[num2str(velocity)],'horizontalAlignment','center','FontName','Formula1','Color',text_color,'FontSize',14)
    end
    text(0.0,2.0,'Velocity','FontSize',22,'FontName','Formula1','Color',text_color,'horizontalAlignment','center');
    text(0.0,0.0,[num2str(a_x/9.81,'%.1f'),'g'],'FontSize',32,'FontName','Formula1','Color',text_color,'horizontalAlignment','center');
    axis equal
    xlim([-2.0,2.0]);
    ylim([-1.5,2.5]);
    
    
    set(h, 'InvertHardcopy', 'off');
    print(h,['figs/',file_name,'/fig_',num2str(i_frame),'.png'],'-dpng','-r0');
    close(h)
end
end

function plot_comparison(run,run2,s,vehicle,state_names,algebraic_state_names,control_names,output_names,file_name)
mkdir(['figs/',file_name])
background_color =  [13/255, 17/255, 23/255];
text_color = [201,209,217]/255;
blue   = [64 224 208]/255;1.3*[0   0.447000000000000   0.741];
 %[64 224 208]/255
time = 0:1/30.0:min(run.time(end),run2.time(end));
tire_data = readstruct('../limebeer-2014-f1.xml');
for i_frame = 1:numel(time)
    fprintf('frame: %d/%d\n',i_frame,length(time));
    q = [run.front_axle.left_tire.kappa; run.front_axle.right_tire.kappa; run.rear_axle.left_tire.kappa; run.rear_axle.right_tire.kappa;...
        run.chassis.velocity.x; run.chassis.velocity.y; run.chassis.omega.z; run.time; run.road.lateral_displacement; run.road.track_heading_angle];
    qa = [run.chassis.Fz_fl; run.chassis.Fz_fr; run.chassis.Fz_rl; run.chassis.Fz_rr];
    u = [run.front_axle.steering_angle; run.rear_axle.boost; run.chassis.throttle; run.chassis.brake_bias];
    
    q_i = interp1(run.time,q',time(i_frame));
    qa_i = interp1(run.time,qa',time(i_frame));
    u_i = interp1(run.time,u',time(i_frame));
    s_i = interp1(run.time,s(1:numel(run.time)),time(i_frame));
    
    q2 = [run2.front_axle.left_tire.kappa; run2.front_axle.right_tire.kappa; run2.rear_axle.left_tire.kappa; run2.rear_axle.right_tire.kappa;...
        run2.chassis.velocity.x; run2.chassis.velocity.y; run2.chassis.omega.z; run2.time; run2.road.lateral_displacement; run2.road.track_heading_angle];
    qa2 = [run2.chassis.Fz_fl; run2.chassis.Fz_fr; run2.chassis.Fz_rl; run2.chassis.Fz_rr];
    u2 = [run2.front_axle.steering_angle; run2.rear_axle.boost; run2.chassis.throttle; run2.chassis.brake_bias];
    
    q2_i = interp1(run2.time,q2',time(i_frame));
    qa2_i = interp1(run2.time,qa2',time(i_frame));
    u2_i = interp1(run2.time,u2',time(i_frame));
    s2_i = interp1(run2.time,s(1:numel(run.time)),time(i_frame));
    
    screen_size = get(0,'ScreenSize');
    h = figure('Visible','off','Position',[0 0 screen_size(3) 1080.0/1920.0*screen_size(3)]);
    hold on
    
    % (1) Plot track
    kerb_color_length = 2.5;
    for i = -40:400
        patch([kerb_color_length*2*i,kerb_color_length*(2*i+1),kerb_color_length*(2*i+1),kerb_color_length*2*i],[4,4,4.5,4.5],[1,0,0]) ;
        patch([kerb_color_length*2*i,kerb_color_length*(2*i+1),kerb_color_length*(2*i+1),kerb_color_length*2*i],-[4,4,4.5,4.5],[1,0,0]) ;
    end
    for i = -40:400
        patch([kerb_color_length*(2*i+1), kerb_color_length*(2*i+2),kerb_color_length*(2*i+2),kerb_color_length*(2*i+1)],[4,4,4.5,4.5],[1,1,1]) ;
        patch([kerb_color_length*(2*i+1), kerb_color_length*(2*i+2),kerb_color_length*(2*i+2),kerb_color_length*(2*i+1)],-[4,4,4.5,4.5],[1,1,1]) ;
    end
    
    patch([-100,1100,1100,-100],[-4,-4,4,4],[129/255, 149/255, 179/255]);
    
    axis equal
    h.Color = [13/255, 17/255, 23/255];
    h.CurrentAxes.Visible = 'off';
    ax = gca;
    ax.Position = [0,0,1,1];
    % (n) Draw car
    x_rr = calllib("libfastestlapc","vehicle_get_output",vehicle,q_i,qa_i,u_i,s_i,'rear_axle.right_tire.x');
    y_rr = calllib("libfastestlapc","vehicle_get_output",vehicle,q_i,qa_i,u_i,s_i,'rear_axle.right_tire.y');
    psi = q_i(end);
    
    plot_f1(x_rr,-y_rr-2,rad2deg(psi)+180,1.6+1.8, 0.73*2,'redbull');
    text(x_rr+1.6+1.8+5.0,-y_rr-2+0.73,'Front brake only','backgroundColor',[1,0,0],'FontName','Formula1','FontSize',20,'horizontalAlignment','center');
    
    x2_rr = calllib("libfastestlapc","vehicle_get_output",vehicle,q2_i,qa2_i,u2_i,s2_i,'rear_axle.right_tire.x');
    y2_rr = calllib("libfastestlapc","vehicle_get_output",vehicle,q2_i,qa2_i,u2_i,s2_i,'rear_axle.right_tire.y');
    psi2 = q2_i(end);
    
    plot_f1(x2_rr,-y2_rr+2,rad2deg(psi2)+180,1.6+1.8, 0.73*2,'ferrari');
    text(x2_rr+1.6+1.8+5.0,-y2_rr+2+0.73,'Rear brake only','backgroundColor',[1,0,0],'FontName','Formula1','FontSize',20,'horizontalAlignment','center');
    
    plot(1.6+1.8+[x_rr,x2_rr],[0.0,0.0],'.-','Color',text_color,'MarkerSize',30,'LineWidth',2)
    text(1.6+1.8+0.5*(x_rr+x2_rr),0.5,['Distance = ', num2str(x2_rr - x_rr,'%.2f'),' m'],'FontName','Formula1','FontSize',18,'horizontalAlignment','center','Color',text_color);
    % (3) Set camera
    camera_width = 20.0;
    xlim([0.5*(s_i+s2_i)-1.777*camera_width*0.5,0.5*(s_i+s2_i)+1.777*camera_width*0.5])
    ylim([-5,-5+camera_width])
    
    % (4) Plot the pacejka model and the point
    ax_pacejka = axes('Position',[0.1 0.55 1080.0/1920.0*0.4 0.4]);
    hold on
    kappa_plot = linspace(-0.5,0.5,1000);
    plot(kappa_plot,pacejka_model_longitudinal(-qa_i(1)*660*9.81,kappa_plot,tire_data),'LineWidth',2,'Color',blue);

    
    plot(kappa_plot,pacejka_model_longitudinal(-qa2_i(1)*660*9.81,kappa_plot,tire_data),'--','LineWidth',2,'Color',[1,0,0]);
    plot(q_i(1),pacejka_model_longitudinal(-qa_i(1)*660*9.81,q_i(1),tire_data),'o','MarkerSize',24,'Color',blue,'LineWidth',4);
    plot(q_i(1),pacejka_model_longitudinal(-qa_i(1)*660*9.81,q_i(1),tire_data),'.','MarkerSize',24,'Color',blue,'LineWidth',4);
    plot(q2_i(1),pacejka_model_longitudinal(-qa2_i(1)*660*9.81,q2_i(1),tire_data),'o','MarkerSize',24,'Color',[1,0,0],'LineWidth',4);
    plot(q2_i(1),pacejka_model_longitudinal(-qa2_i(1)*660*9.81,q2_i(1),tire_data),'.','MarkerSize',24,'Color',[1,0,0],'LineWidth',4);
    
    ax_pacejka.Color = background_color;
    ax_pacejka.XAxis.Color = text_color;
    ax_pacejka.YAxis.Color = text_color;
    ax_pacejka.FontSize = 14;
    ax_pacejka.XMinorTick = 'on';
    ax_pacejka.YMinorTick = 'on';
    ax_pacejka.LineWidth = 2;
    ax_pacejka.YAxis.Exponent = 0;
    xlabel('front tire longitudinal slip, kappa [-]','FontSize',20);
    ylabel('front tire longitudinal force, Fx [N]','FontSize',20);
    box on
    grid on
    ax_pacejka.XMinorGrid = 'on';
    ax_pacejka.YMinorGrid = 'on';
    set(gca,'MinorGridLineStyle','-')
    ylim([-12000,12000]);
    
    % (4) Plot the pacejka model and the point
    ax_pacejka = axes('Position',[0.4 0.55 1080.0/1920.0*0.4 0.4]);
    hold on
    kappa_plot = linspace(-0.5,0.5,1000);
    plot(kappa_plot,pacejka_model_longitudinal(-qa_i(3)*660*9.81,kappa_plot,tire_data),'LineWidth',2,'Color',blue);
    plot(kappa_plot,pacejka_model_longitudinal(-qa2_i(3)*660*9.81,kappa_plot,tire_data),'--','LineWidth',2,'Color',[1,0,0]);
    plot(q_i(3),pacejka_model_longitudinal(-qa_i(3)*660*9.81,q_i(3),tire_data),'o','MarkerSize',24,'Color',blue,'LineWidth',4);
    plot(q_i(3),pacejka_model_longitudinal(-qa_i(3)*660*9.81,q_i(3),tire_data),'.','MarkerSize',24,'Color',blue,'LineWidth',4);
    

    plot(q2_i(3),pacejka_model_longitudinal(-qa2_i(3)*660*9.81,q2_i(3),tire_data),'o','MarkerSize',24,'Color',[1,0,0],'LineWidth',4);
    plot(q2_i(3),pacejka_model_longitudinal(-qa2_i(3)*660*9.81,q2_i(3),tire_data),'.','MarkerSize',24,'Color',[1,0,0],'LineWidth',4);
    
    ax_pacejka.Color = background_color;
    ax_pacejka.XAxis.Color = text_color;
    ax_pacejka.YAxis.Color = text_color;
    ax_pacejka.FontSize = 14;
    ax_pacejka.XMinorTick = 'on';
    ax_pacejka.YMinorTick = 'on';
    ax_pacejka.LineWidth = 2;
    ax_pacejka.YAxis.Exponent = 0;
    xlabel('rear tire longitudinal slip, kappa [-]','FontSize',20);
    ylabel('rear tire longitudinal force, Fx [N]','FontSize',20);
    box on
    grid on
    ax_pacejka.XMinorGrid = 'on';
    ax_pacejka.YMinorGrid = 'on';
    set(gca,'MinorGridLineStyle','-')
    ylim([-12000,12000]);    
    
    % (5) Plot throttle
    ax_throttle = axes('Position',[0.675 0.55 1080.0/1920.0*0.4*0.2 0.4]);
    patch([0,0,1,1,0],[0,-u_i(3)*100,-u_i(3)*100,0,0],blue);
    patch([1,1,2,2,0],[0,-u2_i(3)*100,-u2_i(3)*100,0,0],[1,0,0]);
    ylabel('Brake [%]')
    ax_throttle.Color = background_color;
    ax_throttle.XAxis.Color = text_color;
    ax_throttle.YAxis.Color = text_color;
    ax_throttle.FontSize = 14;
    ax_throttle.XMinorTick = 'on';
    ax_throttle.YMinorTick = 'on';
    ax_throttle.LineWidth = 2;
    ax_throttle.XTick = [];
    ylim([0,100]);
    box on
    
    % Plot velocity in the form of gauges
    ax_velocity = axes('Position',[0.725 0.50 1080.0/1920.0*0.5 0.5]);
    hold on;
    ax_velocity.Visible = 'off';
    patch([ - 1.5*sind(linspace(45,315,50)), -1.2*sind(linspace(315,45,50))],...
        [-1.5*cosd(linspace(45,315,50)), -1.2*cosd(linspace(315,45,50))],text_color);
    patch([ - 1.2*sind(linspace(45,315,50)), -0.9*sind(linspace(315,45,50))],...
        [-1.2*cosd(linspace(45,315,50)), -0.9*cosd(linspace(315,45,50))],text_color);
    
    vel = max(0.0,q_i(5));
    angle_for_velocity = 45 + (315-45)*3.6*vel/350;
    patch([ - 1.5*sind(linspace(45,angle_for_velocity,50)), -1.2*sind(linspace(angle_for_velocity,45,50))],...
        [-1.5*cosd(linspace(45,angle_for_velocity,50)), -1.2*cosd(linspace(angle_for_velocity,45,50))],blue);
    
    vel = max(0.0,q2_i(5));
    angle_for_velocity = 45 + (315-45)*3.6*vel/350;
    patch([ - 1.2*sind(linspace(45,angle_for_velocity,50)), -0.9*sind(linspace(angle_for_velocity,45,50))],...
        [-1.2*cosd(linspace(45,angle_for_velocity,50)), -0.9*cosd(linspace(angle_for_velocity,45,50))],[1,0,0]);
    % Put marks at several velocities
    velocities = 0:25:350;
    
    for velocity = velocities
        angle = 45 + (315-45)*velocity/velocities(end);
        text(-1.75*sind(angle), -1.75*cosd(angle),[num2str(velocity)],'horizontalAlignment','center','FontName','Formula1','Color',text_color,'FontSize',14)
    end
    text(0.0,2.0,'Velocity','FontSize',22,'FontName','Formula1','Color',text_color,'horizontalAlignment','center');
    axis equal
    xlim([-2.0,2.0]);
    ylim([-1.5,2.5]);
    
    
    set(h, 'InvertHardcopy', 'off');
    print(h,['figs/',file_name,'/fig_',num2str(i_frame),'.png'],'-dpng','-r300');
    % Export as GIF
    frame = getframe(1);
    im = frame2im(frame);
    [imind,cm] = rgb2ind(im,256);
    if i_frame == 1
        imwrite(imind,cm,[file_name,'.gif'],'gif', 'Loopcount',inf);
    elseif i_frame == numel(time)
        for iii = 1 : 30
        imwrite(imind,cm,[file_name,'.gif'],'gif','WriteMode','append','DelayTime',1/30.0);
        end
    else
        imwrite(imind,cm,[file_name,'.gif'],'gif','WriteMode','append','DelayTime',1/30.0);
    end
    close(h)
end
end

function [Fx,kappa_max_real] = pacejka_model_longitudinal(Fz,kappa,tire_data)
Fz_1 = tire_data.rear_tire.reference_load_1.Text;
Fz_2 = tire_data.rear_tire.reference_load_2.Text;
mu_x_1 = tire_data.rear_tire.mu_x_max_1;
mu_x_2 = tire_data.rear_tire.mu_x_max_2;
kappa_max_1 = tire_data.rear_tire.kappa_max_1;
kappa_max_2 = tire_data.rear_tire.kappa_max_2;
Qx = tire_data.rear_tire.Qx;
Sx = pi/(2*atan(Qx));
mu_x_max = (Fz-Fz_1)*(mu_x_2-mu_x_1)/(Fz_2-Fz_1) + mu_x_1;
kappa_max = (Fz-Fz_1)*(kappa_max_2-kappa_max_1)/(Fz_2-Fz_1) + kappa_max_1;
kappa_max_real = kappa_max*0.751247350722835;
Fx = mu_x_max*Fz.*sin(Qx*atan(Sx*kappa./kappa_max));
end
