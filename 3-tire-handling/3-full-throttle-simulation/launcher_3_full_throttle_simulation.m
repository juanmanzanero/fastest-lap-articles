if (~exist('restart','var'))
    restart = true;
end

if (restart)
    clear all
    clc
    close all
    
    prefix = '/Users/juanmanzanero/Documents/software/fastest-lap';
    lib_suffix = 'dylib';
    fastest_lap_version = '0.3.5';
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
    
    % (4) Run steady-state computation at 30 km/h
    [q0,qa0,u0] = calllib('libfastestlapc','steady_state',zeros(1,n_state), zeros(1,n_algebraic), zeros(1,n_control), vehicle, 50.0/3.6, 0.0, 0.0);
    
    % (4.1) Store it in the internal memory
    calllib('libfastestlapc','create_vector','run/trim_at_30kph/state',n_state,q0);
    calllib('libfastestlapc','create_vector','run/trim_at_30kph/algebraic_state',n_algebraic,qa0);
    calllib('libfastestlapc','create_vector','run/trim_at_30kph/control',n_control,u0);
    
    % (5) Run optimal acceleration
    
    % (5.1) Write options
    options =          '<options>';
    options = [options,    '<closed_simulation> false </closed_simulation>'];
    options = [options,    '<initial_condition>'];
    options = [options,    '    <q  from_table="run/trim_at_30kph/state"/>'];
    options = [options,    '    <qa from_table="run/trim_at_30kph/algebraic_state"/>'];
    options = [options,    '    <u  from_table="run/trim_at_30kph/control"/>'];
    options = [options,    '</initial_condition>'];
    options = [options,    '<control_variables>'];
    options = [options,    '    <chassis.throttle optimal_control_type="full-mesh">'];
    options = [options,    '        <dissipation> 1.0e0 </dissipation>'];
    options = [options,    '    </chassis.throttle>'];
    options = [options,    '</control_variables>'];
    options = [options,    '<output_variables>'];
    options = [options,    '    <prefix>run/</prefix>'];
    options = [options,    '</output_variables>'];
    options = [options,    '<sigma> 0.5 </sigma>'];
    options = [options,'</options>'];
    
    % (5.2) Run
    fprintf('[INFO] launcher_3_full_throttle_simulation -> [start] optimal laptime simulation\n');
    calllib("libfastestlapc","optimal_laptime",vehicle,track,length(s),s,options);
    fprintf('[INFO] launcher_3_full_throttle_simulation -> [end] optimal laptime simulation\n');
    
    % (5.3) Download
    run = download_vectors('run/', {state_names{:}, algebraic_state_names{:}, control_names{:}}, n_points+1);
    calllib('libfastestlapc','delete_variable','run/*');
    restart = false;
    
    % (6) Emulate a bad driver: a driver that tries to pushes to much,
    % backs off, then pushes back, etc
    q = repmat(q0',1,numel(s));
    qa = repmat(qa0',1,numel(s));
    u = repmat(u0',1,numel(s));
    
    % Go full throttle
    options =          '<options>';
    options = [options,    '<sigma> 0.5 </sigma>'];
    options = [options,    '<!--relaxation_factor> 1.0e-1 </relaxation_factor-->'];
    options = [options,'</options>'];
    
    
    u(3,:) = min(1.01*run.chassis.throttle,1.0);
    u(3,1:10) = run.chassis.throttle(1:10);
    for i = 1 : numel(s)-1
        if ( q(3,i) > 0.1 )
            u(3,i+1:i+1+15) = 0.8*run.chassis.throttle(i+1:i+1+15);
        end
        [q(:,i+1), qa(:,i+1)] = calllib('libfastestlapc','propagate_vehicle',q(:,i),qa(:,i), u(:,i), vehicle, track, s(i), s(i+1)-s(i), u(:,i+1), true, options);
    end
    
    run_bad_driver = transform_run(struct('q',q,'qa',qa,'u',u),state_names,algebraic_state_names,control_names);
    
    % (7) Run a silly test problem
    q = repmat(q0',1,148);
    qa = repmat(qa0',1,148);
    u = repmat(u0',1,148);
    
    options =          '<options>';
    options = [options,    '<sigma> 0.5 </sigma>'];
    options = [options,'</options>'];
    
    
    u(3,:) = 1.0;
    u(3,1:10) = run.chassis.throttle(1:10);
    for i = 1 : 147
        [q(:,i+1), qa(:,i+1)] = calllib('libfastestlapc','propagate_vehicle',q(:,i),qa(:,i), u(:,i), vehicle, track, s(i), s(i+1)-s(i), u(:,i+1), true, options);
    end
    run_silly = transform_run(struct('q',q,'qa',qa,'u',u),state_names,algebraic_state_names,control_names);
    
    save('run','run','run_silly','run_bad_driver','s','vehicle','prefix','track','lib_suffix','fastest_lap_version','state_names','algebraic_state_names','control_names','output_names');
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
% (1) Create GIF with the car that goes wild in the throttle
plot_silly_run(run_silly,s(1:numel(run_silly.time)),vehicle,state_names,algebraic_state_names,control_names,output_names);


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

function plot_silly_run(run,s,vehicle,state_names,algebraic_state_names,control_names,output_names)

time = 0:1/30.0:run.time(end);



for i_frame = 1:195
    fprintf('frame: %d/195\n',i_frame);
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
    for i = -40:40
        patch([kerb_color_length*2*i,kerb_color_length*(2*i+1),kerb_color_length*(2*i+1),kerb_color_length*2*i],[5,5,5.5,5.5],[1,0,0]) ;
        patch([kerb_color_length*2*i,kerb_color_length*(2*i+1),kerb_color_length*(2*i+1),kerb_color_length*2*i],-[5,5,5.5,5.5],[1,0,0]) ;
    end
    for i = -40:40
        patch([kerb_color_length*(2*i+1), kerb_color_length*(2*i+2),kerb_color_length*(2*i+2),kerb_color_length*(2*i+1)],[5,5,5.5,5.5],[1,1,1]) ;
        patch([kerb_color_length*(2*i+1), kerb_color_length*(2*i+2),kerb_color_length*(2*i+2),kerb_color_length*(2*i+1)],-[5,5,5.5,5.5],[1,1,1]) ;
    end
    
    patch([-100,1100,1100,-100],[-5,-5,5,5],[129/255, 149/255, 179/255]);
    
    
    
    axis equal
    h.Color = [13/255, 17/255, 23/255];
    h.CurrentAxes.Visible = 'off';
    ax = gca;
    ax.Position = [0,0,1,1];
    % (n) Draw car
    x_rr = calllib("libfastestlapc","vehicle_get_output",vehicle,q_i,qa_i,u_i,s_i,'rear_axle.right_tire.x');
    y_rr = calllib("libfastestlapc","vehicle_get_output",vehicle,q_i,qa_i,u_i,s_i,'rear_axle.right_tire.y');
    psi = q_i(end);
    
    plot_f1(x_rr,-y_rr,rad2deg(psi)+180,1.6+1.8, 0.73*2,'redbull');
    
    
    % (3) Set camera
    camera_width = 20.0;
    xlim([s_i-1.777*camera_width*0.5,s_i+1.777*camera_width*0.5])
    ylim([-camera_width*0.5,camera_width*0.5])
    
    % Export as GIF
    frame = getframe(1);
    im = frame2im(frame);
    [imind,cm] = rgb2ind(im,256);
    if i_frame == 1
        imwrite(imind,cm,'1_silly_throttle.gif','gif', 'Loopcount',inf);
    else
        imwrite(imind,cm,'1_silly_throttle.gif','gif','WriteMode','append','DelayTime',1/30.0);
    end
    close(h)    
end
end