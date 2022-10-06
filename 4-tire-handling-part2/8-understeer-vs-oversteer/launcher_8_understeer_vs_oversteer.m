%% Block 1: Computations
set(groot,'defaultAxesFontName','Formula1 Display Regular');
%clear all
if ~exist('restart','var') | restart
    clear all
    close all
    global fastest_lap

    fastest_lap = 'flap';
    fastest_lap_path = 'C:\Users\jmanz\Documents\software\fastest-lap-vs\fastest-lap';
    fastest_lap_library_path = 'C:\Users\jmanz\Documents\software\fastest-lap-vs\vs\x64\Release';
    fastest_lap_version = '-0.4';
    fastest_lap_suffix = 'dll';
    vehicle_database = 'neutral-car.xml';
    addpath([fastest_lap_path, '/src/main/matlab']);
    x_pressure_center_understeer = -0.3;
    x_pressure_center_oversteer  = 0.0;
    x_pressure_center_neutral = -0.14;

    kmh = 1/3.6;
    n_max_char = 100;

    dtor = onCleanup(@()(clean(fastest_lap)));

    loadlibrary([fastest_lap_library_path,'\libfastestlapc',fastest_lap_version,'.',fastest_lap_suffix],...
        [fastest_lap_path,'/src/main/c/fastestlapc.h'],'alias',fastest_lap);

    calllib(fastest_lap,'set_print_level',0);

    % (1) Get information from the car
    [n_state, n_algebraic, n_control, n_outputs] = calllib(fastest_lap,'vehicle_type_get_sizes',0,0,0,0,'f1-3dof');
    [key_name, state_names, algebraic_state_names, control_names, output_names] = calllib(fastest_lap,'vehicle_type_get_names',blanks(n_max_char),repmat({blanks(n_max_char)},1,n_state),repmat({blanks(n_max_char)},1,n_algebraic),repmat({blanks(n_max_char)},1,n_control),repmat({blanks(n_max_char)},1,n_outputs),n_max_char,'f1-3dof');

    % (2) Load vehicle
    vehicle = 'car';
    calllib(fastest_lap,'create_vehicle_from_xml',vehicle,vehicle_database);
    calllib(fastest_lap,'vehicle_set_parameter',vehicle,'vehicle/chassis/pressure_center/x',x_pressure_center_neutral);
    % (3) Create a circular track
    n_points = 50;
    [track_limits.r_left, track_limits.r_right] = create_circular_track(0.015,n_points,12,true);

    % (3.1) Load it
    track = 'track';
    calllib(fastest_lap,'create_track_from_xml',track,'circular_track.xml');
    arclength = calllib(fastest_lap,'track_download_data',zeros(1,n_points), track, n_points, 'arclength');

    % (4) Run an optimal laptime
    fprintf('Neutral car -----------------\n');
    options =          '<options>';
    options = [options,'    <control_variables>'];
    options = [options,'        <chassis.throttle optimal_control_type="constant"/>'];
    options = [options,'        <front-axle.steering-angle optimal_control_type="constant"/>'];
    options = [options,'    </control_variables>'];
    options = [options,'    <output_variables>'];
    options = [options,'        <prefix> run/ </prefix>'];
    options = [options,'    </output_variables>'];
    options = [options,'</options>'];

    calllib(fastest_lap,'optimal_laptime',vehicle,track,n_points,arclength,options);

    % (4.1) Get data: the right side is the loaded one
    run.time = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run/time');
    run.u = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run/chassis.velocity.x');
    run.v = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run/chassis.velocity.y');
    run.pos.x = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run/chassis.position.x');
    run.pos.y = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run/chassis.position.y');
    run.lateral_displacement = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run/road.lateral-displacement');
    run.track_heading_angle = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run/road.track-heading-angle');
    run.omega = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run/chassis.omega.z');
    run.Fz_fl = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run/chassis.Fz_fl');
    run.Fz_fr = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run/chassis.Fz_fr');
    run.Fz_rl = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run/chassis.Fz_rl');
    run.Fz_rr = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run/chassis.Fz_rr');
    run.lambda_fr = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run/front-axle.right-tire.lambda');
    run.lambda_rr = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run/rear-axle.right-tire.lambda');
    run.kappa_fr = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run/front-axle.right-tire.kappa');
    run.kappa_rr = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run/rear-axle.right-tire.kappa');
    run.lambda_fl = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run/front-axle.left-tire.lambda');
    run.lambda_rl = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run/rear-axle.left-tire.lambda');
    run.kappa_fl = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run/front-axle.left-tire.kappa');
    run.kappa_rl = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run/rear-axle.left-tire.kappa');
    run.delta = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run/front-axle.steering-angle');
    run.throttle = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run/chassis.throttle');
    run.rho_fl = sqrt((run.lambda_fl/deg2rad(9)).^2 + (run.kappa_fl).^2);
    run.rho_fr = sqrt((run.lambda_fr/deg2rad(9)).^2 + (run.kappa_fr).^2);
    run.rho_rl = sqrt((run.lambda_rl/deg2rad(9)).^2 + (run.kappa_rl).^2);
    run.rho_rr = sqrt((run.lambda_rr/deg2rad(9)).^2 + (run.kappa_rr).^2);

    fprintf('Velocity: %gkm/h     Throttle: %g\n',run.u(1)*3.6,run.throttle(1)*100);
    fprintf('Fz front: %gN        Fz  rear: %gN\n',run.Fz_fr(1), run.Fz_rr(1));
    fprintf('(outer tires) rho front: %g%%      rho rear: %g%%\n',run.rho_fr(1)/0.75124*100, run.rho_rr(1)/0.75124*100);
    fprintf('(inner tires) rho front: %g%%      rho rear: %g%%\n',run.rho_fl(1)/0.75124*100, run.rho_rl(1)/0.75124*100);

    % (5) Run an optimal laptime of an understeering car
    fprintf('\nUndersteering car -----------------\n');
    vehicle_understeer = 'vehicle_understeer';

    calllib(fastest_lap,'copy_variable',vehicle,vehicle_understeer);
    calllib(fastest_lap,'vehicle_set_parameter',vehicle_understeer,'vehicle/chassis/pressure_center/x',x_pressure_center_understeer);

    options =          '<options>';
    options = [options,'    <control_variables>'];
    options = [options,'        <chassis.throttle optimal_control_type="constant"/>'];
    options = [options,'        <front-axle.steering-angle optimal_control_type="constant"/>'];
    options = [options,'    </control_variables>'];
    options = [options,'    <output_variables>'];
    options = [options,'        <prefix> run_understeer/ </prefix>'];
    options = [options,'    </output_variables>'];
    options = [options,'</options>'];

    calllib(fastest_lap,'optimal_laptime',vehicle_understeer,track,n_points,arclength,options);

    % (5.1) Get data: the right side is the loaded one
    run_understeer.time = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run_understeer/time');
    run_understeer.u = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run_understeer/chassis.velocity.x');
    run_understeer.v = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run_understeer/chassis.velocity.y');
    run_understeer.pos.x = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run_understeer/chassis.position.x');
    run_understeer.pos.y = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run_understeer/chassis.position.y');
    run_understeer.lateral_displacement = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run_understeer/road.lateral-displacement');
    run_understeer.track_heading_angle = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run_understeer/road.track-heading-angle');
    run_understeer.omega = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run_understeer/chassis.omega.z');
    run_understeer.Fz_fl = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run_understeer/chassis.Fz_fl');
    run_understeer.Fz_fr = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run_understeer/chassis.Fz_fr');
    run_understeer.Fz_rl = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run_understeer/chassis.Fz_rl');
    run_understeer.Fz_rr = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run_understeer/chassis.Fz_rr');
    run_understeer.lambda_fr = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run_understeer/front-axle.right-tire.lambda');
    run_understeer.lambda_rr = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run_understeer/rear-axle.right-tire.lambda');
    run_understeer.kappa_fr = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run_understeer/front-axle.right-tire.kappa');
    run_understeer.kappa_rr = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run_understeer/rear-axle.right-tire.kappa');
    run_understeer.lambda_fl = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run_understeer/front-axle.left-tire.lambda');
    run_understeer.lambda_rl = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run_understeer/rear-axle.left-tire.lambda');
    run_understeer.kappa_fl = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run_understeer/front-axle.left-tire.kappa');
    run_understeer.kappa_rl = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run_understeer/rear-axle.left-tire.kappa');
    run_understeer.delta = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run_understeer/front-axle.steering-angle');
    run_understeer.throttle = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run_understeer/chassis.throttle');
    run_understeer.rho_fl = sqrt((run_understeer.lambda_fl/deg2rad(9)).^2 + (run_understeer.kappa_fl).^2);
    run_understeer.rho_fr = sqrt((run_understeer.lambda_fr/deg2rad(9)).^2 + (run_understeer.kappa_fr).^2);
    run_understeer.rho_rl = sqrt((run_understeer.lambda_rl/deg2rad(9)).^2 + (run_understeer.kappa_rl).^2);
    run_understeer.rho_rr = sqrt((run_understeer.lambda_rr/deg2rad(9)).^2 + (run_understeer.kappa_rr).^2);

    fprintf('Velocity: %gkm/h     Throttle: %g\n',run_understeer.u(1)*3.6,run_understeer.throttle(1)*100);
    fprintf('Fz front: %gN        Fz  rear: %gN\n',run_understeer.Fz_fr(1), run_understeer.Fz_rr(1));
    fprintf('(outer tires) rho front: %g%%      rho rear: %g%%\n',run_understeer.rho_fr(1)/0.75124*100, run_understeer.rho_rr(1)/0.75124*100);
    fprintf('(inner tires) rho front: %g%%      rho rear: %g%%\n',run_understeer.rho_fl(1)/0.75124*100, run_understeer.rho_rl(1)/0.75124*100);

    % (5.2) Run a simulation with the 'steady-state' controls
    ds_simulation = 1.0;
    n_timesteps = 89;
    run_understeer.q0 = [run_understeer.kappa_fl(1), run_understeer.kappa_fr(1), run_understeer.kappa_rl(1), run_understeer.kappa_rr(1), run_understeer.u(1), run_understeer.v(1), run_understeer.omega(1), 0, run_understeer.lateral_displacement(1), run_understeer.track_heading_angle(1)];
    run_understeer.qa0 = [run_understeer.Fz_fl(1), run_understeer.Fz_fr(1), run_understeer.Fz_rl(1), run_understeer.Fz_rr(1)];
    run_understeer.controls0 = [run_understeer.delta(1), 0.0, run_understeer.throttle(1), 0.6];

    run_understeer.q = repmat(run_understeer.q0(:),1,n_timesteps);
    run_understeer.qa = repmat(run_understeer.qa0(:),1,n_timesteps);
    run_understeer.controls = repmat(run_understeer.controls0(:),1,n_timesteps);

    options =          '<options>';
    options = [options,    '<sigma> 0.5 </sigma>'];
    options = [options,    '<error_tolerance> 1.0e-10 </error_tolerance>'];
    options = [options,'</options>'];
    fprintf('[run_understeer simulation] -> start\n');
    current_arclength = 0.0;
    for i = 1 : n_timesteps-1
        [run_understeer.q(:,i+1), run_understeer.qa(:,i+1)] = calllib(fastest_lap,'propagate_vehicle',run_understeer.q(:,i),run_understeer.qa(:,i), run_understeer.controls(:,i), vehicle_understeer, track, current_arclength, ds_simulation, run_understeer.controls(:,i+1), true, options);
        current_arclength = current_arclength + ds_simulation;

        % After 1 second, increase the steering by 10%
        if run_understeer.q(8,i+1) > 1.0
            run_understeer.controls(1,i+1:end) = run_understeer.controls(1,i+1:end)*1.1;
        end
    end
    fprintf('[run_understeer simulation] -> finish\n');

    % (6) Run an optimal laptime of an oversteering car
    fprintf('\nOversteering car -----------------\n');
    vehicle_oversteer = 'vehicle_oversteer';

    calllib(fastest_lap,'copy_variable',vehicle,vehicle_oversteer);
    calllib(fastest_lap,'vehicle_set_parameter',vehicle_oversteer,'vehicle/chassis/pressure_center/x',x_pressure_center_oversteer);

    options =          '<options>';
    options = [options,'    <control_variables>'];
    options = [options,'        <chassis.throttle optimal_control_type="constant"/>'];
    options = [options,'        <front-axle.steering-angle optimal_control_type="constant"/>'];
    options = [options,'    </control_variables>'];
    options = [options,'    <output_variables>'];
    options = [options,'        <prefix> run_oversteer/ </prefix>'];
    options = [options,'    </output_variables>'];
    options = [options,'</options>'];

    calllib(fastest_lap,'optimal_laptime',vehicle_oversteer,track,n_points,arclength,options);

    % (6.1) Get data: the right side is the loaded one
    run_oversteer.time = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run_oversteer/time');
    run_oversteer.u = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run_oversteer/chassis.velocity.x');
    run_oversteer.v = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run_oversteer/chassis.velocity.y');
    run_oversteer.pos.x = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run_oversteer/chassis.position.x');
    run_oversteer.pos.y = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run_oversteer/chassis.position.y');
    run_oversteer.lateral_displacement = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run_oversteer/road.lateral-displacement');
    run_oversteer.track_heading_angle = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run_oversteer/road.track-heading-angle');
    run_oversteer.omega = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run_oversteer/chassis.omega.z');
    run_oversteer.Fz_fl = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run_oversteer/chassis.Fz_fl');
    run_oversteer.Fz_fr = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run_oversteer/chassis.Fz_fr');
    run_oversteer.Fz_rl = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run_oversteer/chassis.Fz_rl');
    run_oversteer.Fz_rr = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run_oversteer/chassis.Fz_rr');
    run_oversteer.lambda_fr = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run_oversteer/front-axle.right-tire.lambda');
    run_oversteer.lambda_rr = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run_oversteer/rear-axle.right-tire.lambda');
    run_oversteer.kappa_fr = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run_oversteer/front-axle.right-tire.kappa');
    run_oversteer.kappa_rr = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run_oversteer/rear-axle.right-tire.kappa');
    run_oversteer.lambda_fl = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run_oversteer/front-axle.left-tire.lambda');
    run_oversteer.lambda_rl = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run_oversteer/rear-axle.left-tire.lambda');
    run_oversteer.kappa_fl = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run_oversteer/front-axle.left-tire.kappa');
    run_oversteer.kappa_rl = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run_oversteer/rear-axle.left-tire.kappa');
    run_oversteer.delta = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run_oversteer/front-axle.steering-angle');
    run_oversteer.throttle = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run_oversteer/chassis.throttle');
    run_oversteer.rho_fl = sqrt((run_oversteer.lambda_fl/deg2rad(9)).^2 + (run_oversteer.kappa_fl).^2);
    run_oversteer.rho_fr = sqrt((run_oversteer.lambda_fr/deg2rad(9)).^2 + (run_oversteer.kappa_fr).^2);
    run_oversteer.rho_rl = sqrt((run_oversteer.lambda_rl/deg2rad(9)).^2 + (run_oversteer.kappa_rl).^2);
    run_oversteer.rho_rr = sqrt((run_oversteer.lambda_rr/deg2rad(9)).^2 + (run_oversteer.kappa_rr).^2);

    fprintf('Velocity: %gkm/h     Throttle: %g\n',run_oversteer.u(1)*3.6,run_oversteer.throttle(1)*100);
    fprintf('Fz front: %gN        Fz  rear: %gN\n',run_oversteer.Fz_fr(1), run_oversteer.Fz_rr(1));
    fprintf('(outer tires) rho front: %g%%      rho rear: %g%%\n',run_oversteer.rho_fr(1)/0.75124*100, run_oversteer.rho_rr(1)/0.75124*100);
    fprintf('(inner tires) rho front: %g%%      rho rear: %g%%\n',run_oversteer.rho_fl(1)/0.75124*100, run_oversteer.rho_rl(1)/0.75124*100);

    % (6.2) Run a simulation with the 'steady-state' controls
    ds_simulation = 1.0;
    n_timesteps = 120;
    run_oversteer.q0 = [run_oversteer.kappa_fl(1), run_oversteer.kappa_fr(1), run_oversteer.kappa_rl(1), run_oversteer.kappa_rr(1), run_oversteer.u(1), run_oversteer.v(1), run_oversteer.omega(1), 0, run_oversteer.lateral_displacement(1), run_oversteer.track_heading_angle(1)];
    run_oversteer.qa0 = [run_oversteer.Fz_fl(1), run_oversteer.Fz_fr(1), run_oversteer.Fz_rl(1), run_oversteer.Fz_rr(1)];
    run_oversteer.controls0 = [run_oversteer.delta(1), 0.0, run_oversteer.throttle(1), 0.6];

    run_oversteer.q = repmat(run_oversteer.q0(:),1,n_timesteps);
    run_oversteer.qa = repmat(run_oversteer.qa0(:),1,n_timesteps);
    run_oversteer.controls = repmat(run_oversteer.controls0(:),1,n_timesteps);

    options =          '<options>';
    options = [options,    '<sigma> 0.5 </sigma>'];
    options = [options,    '<error_tolerance> 1.0e-10 </error_tolerance>'];
    options = [options,'</options>'];
    fprintf('[run_oversteer simulation] -> start\n');
    current_arclength = 0.0;
    for i = 1 : n_timesteps-1
        [run_oversteer.q(:,i+1), run_oversteer.qa(:,i+1)] = calllib(fastest_lap,'propagate_vehicle',run_oversteer.q(:,i),run_oversteer.qa(:,i), run_oversteer.controls(:,i), vehicle_oversteer, track, current_arclength, ds_simulation, run_oversteer.controls(:,i+1), true, options);
        current_arclength = current_arclength + ds_simulation;

        % After 1 second, increase the steering by 10%
        if run_oversteer.q(8,i+1) > 1.0
            run_oversteer.controls(1,i+1:end) = run_oversteer.controls(1,i+1:end)*1.1;
        end
    end
    fprintf('[run_oversteer simulation] -> finish\n');

    restart = false;
end

%% Block 2: Representations
case_to_plot = 'neutral';
representation_type = 'image';
skin = 'mercedes';


if strcmpi(case_to_plot,'understeer')
    run_to_plot = run_understeer;
    vehicle_to_plot = vehicle_understeer;
    figure_title = 'Understeer';
elseif strcmpi(case_to_plot,'neutral')
    run_to_plot = run;
    vehicle_to_plot = vehicle;
    figure_title = 'Neutral';
else
    error('yeah');
end


if strcmpi(representation_type,'image')
    represent_image(run_to_plot, skin, vehicle_to_plot,arclength(1),figure_title)
end

function represent_image(run,skin,vehicle,arclength,figure_title)
global fastest_lap

text_color =  [13/255, 17/255, 23/255];
background_color = [1,1,1];
asphalt_color   = [129/255, 149/255, 179/255];
track_curvature = 0.015;
n_points_track = 5000;

[track_limits.r_left, track_limits.r_right] = create_circular_track(track_curvature,n_points_track,12,false);

h = figure('Position',[237         191        1574         787]);
h.Color = background_color;
hold on
ax = gca;
ax.Visible = 'off';
patch([track_limits.r_left(1,:),track_limits.r_right(1,:)],[track_limits.r_left(2,:),track_limits.r_right(2,:)],asphalt_color,'LineStyle','none')
plot(track_limits.r_left(1,:),track_limits.r_left(2,:),'-k');
plot(track_limits.r_right(1,:),track_limits.r_right(2,:),'-k');

n_kerb = 150;
n_points_per_kerb = 50;
kerb_theta = linspace(0,2*pi,n_kerb*n_points_per_kerb) - arclength/track_curvature;

for i = 1 : n_kerb-1
    if mod(i,2) == 0
        kerb_color = [1,0.3,0.3];
    else
        kerb_color = [1,1,1];
    end
    patch([cos(kerb_theta((i-1)*n_points_per_kerb+1:i*n_points_per_kerb+1)),0.98*cos(kerb_theta(i*n_points_per_kerb+1:-1:(i-1)*n_points_per_kerb+1))]*(1/track_curvature-6),...
        [sin(kerb_theta((i-1)*n_points_per_kerb+1:i*n_points_per_kerb+1)),0.98*sin(kerb_theta(i*n_points_per_kerb+1:-1:(i-1)*n_points_per_kerb+1))]*(1/track_curvature-6),...
        kerb_color...
        )
end
patch([cos(kerb_theta((n_kerb-1)*n_points_per_kerb+1:n_kerb*n_points_per_kerb)),0.98*cos(kerb_theta(n_kerb*n_points_per_kerb:-1:(n_kerb-1)*n_points_per_kerb+1))]*(1/track_curvature-6),...
    [sin(kerb_theta((n_kerb-1)*n_points_per_kerb+1:n_kerb*n_points_per_kerb)),0.98*sin(kerb_theta(n_kerb*n_points_per_kerb:-1:(n_kerb-1)*n_points_per_kerb+1))]*(1/track_curvature-6),...
    [1,0,0]...
    )

q0 = [run.kappa_fl(1), run.kappa_fr(1), run.kappa_rl(1), run.kappa_rr(1), run.u(1), run.v(1), run.omega(1), 0, run.lateral_displacement(1), run.track_heading_angle(1)];
qa0 = [run.Fz_fl(1), run.Fz_fr(1), run.Fz_rl(1), run.Fz_rr(1)];
u0 = [run.delta(1), 0.0, run.throttle(1), 0.6];

% Decision: plot the car always in s = 0, so that the kerb rotates but the
% car does not
x_rr = calllib(fastest_lap,'vehicle_get_output',vehicle,q0,qa0,u0,0,'rear-axle.right-tire.position.x');
y_rr = calllib(fastest_lap,'vehicle_get_output',vehicle,q0,qa0,u0,0,'rear-axle.right-tire.position.y');
x_fr = calllib(fastest_lap,'vehicle_get_output',vehicle,q0,qa0,u0,0,'front-axle.right-tire.position.x');
y_fr = calllib(fastest_lap,'vehicle_get_output',vehicle,q0,qa0,u0,0,'front-axle.right-tire.position.y');
psi  = calllib(fastest_lap,'vehicle_get_output',vehicle,q0,qa0,u0,0,'chassis.attitude.yaw');
x = calllib(fastest_lap,'vehicle_get_output',vehicle,q0,qa0,u0,0,'chassis.position.x');
y = calllib(fastest_lap,'vehicle_get_output',vehicle,q0,qa0,u0,0,'chassis.position.y');

% Get tire forces
F_fl(1) = calllib(fastest_lap,'vehicle_get_output',vehicle,q0,qa0,u0,0,'front-axle.left-tire.force.inertial.x');
F_fl(2) = calllib(fastest_lap,'vehicle_get_output',vehicle,q0,qa0,u0,0,'front-axle.left-tire.force.inertial.y');

F_fr(1) = calllib(fastest_lap,'vehicle_get_output',vehicle,q0,qa0,u0,0,'front-axle.right-tire.force.inertial.x');
F_fr(2) = calllib(fastest_lap,'vehicle_get_output',vehicle,q0,qa0,u0,0,'front-axle.right-tire.force.inertial.y');

F_rl(1) = calllib(fastest_lap,'vehicle_get_output',vehicle,q0,qa0,u0,0,'rear-axle.left-tire.force.inertial.x');
F_rl(2) = calllib(fastest_lap,'vehicle_get_output',vehicle,q0,qa0,u0,0,'rear-axle.left-tire.force.inertial.y');

F_rr(1) = calllib(fastest_lap,'vehicle_get_output',vehicle,q0,qa0,u0,0,'rear-axle.right-tire.force.inertial.x');
F_rr(2) = calllib(fastest_lap,'vehicle_get_output',vehicle,q0,qa0,u0,0,'rear-axle.right-tire.force.inertial.y');

% Get tire velocities in body frame
v_fl(1) = calllib(fastest_lap,'vehicle_get_output',vehicle,q0,qa0,u0,0,'front-axle.left-tire.velocity.x');
v_fl(2) = calllib(fastest_lap,'vehicle_get_output',vehicle,q0,qa0,u0,0,'front-axle.left-tire.velocity.y');

v_fr(1) = calllib(fastest_lap,'vehicle_get_output',vehicle,q0,qa0,u0,0,'front-axle.right-tire.velocity.x');
v_fr(2) = calllib(fastest_lap,'vehicle_get_output',vehicle,q0,qa0,u0,0,'front-axle.right-tire.velocity.y');

v_rl(1) = calllib(fastest_lap,'vehicle_get_output',vehicle,q0,qa0,u0,0,'rear-axle.left-tire.velocity.x');
v_rl(2) = calllib(fastest_lap,'vehicle_get_output',vehicle,q0,qa0,u0,0,'rear-axle.left-tire.velocity.y');

v_rr(1) = calllib(fastest_lap,'vehicle_get_output',vehicle,q0,qa0,u0,0,'rear-axle.right-tire.velocity.x');
v_rr(2) = calllib(fastest_lap,'vehicle_get_output',vehicle,q0,qa0,u0,0,'rear-axle.right-tire.velocity.y');

% Get lambda
lambda_fl = calllib(fastest_lap,'vehicle_get_output',vehicle,q0,qa0,u0,0,'front-axle.left-tire.lambda');
lambda_fr = calllib(fastest_lap,'vehicle_get_output',vehicle,q0,qa0,u0,0,'front-axle.right-tire.lambda');
lambda_rl = calllib(fastest_lap,'vehicle_get_output',vehicle,q0,qa0,u0,0,'rear-axle.left-tire.lambda');
lambda_rr = calllib(fastest_lap,'vehicle_get_output',vehicle,q0,qa0,u0,0,'rear-axle.right-tire.lambda');

kappa_fl = q0(1);
kappa_fr = q0(2);
kappa_rl = q0(3);
kappa_rr = q0(4);

plot_f1(x_rr,-y_rr,rad2deg(psi)+180,3.6,0.73*2,skin);

draw_arrow([x_fr,x_fr + F_fr(1)/(795*9.81)*2],[-y_fr,-y_fr-F_fr(2)/(795*9.81)*2],[0,1,0]);
draw_arrow([x_rr,x_rr + F_rr(1)/(795*9.81)*2],[-y_rr,-y_rr-F_rr(2)/(795*9.81)*2],[0,1,0]);

axis equal
xlim([1/track_curvature-8,1/track_curvature+22])

annotation('textbox',[0.39,0.85,0.15,0.08],'String',figure_title,'FontName','Formula1 Display Bold','FontSize',25,'BackgroundColor',text_color,'Color',background_color,'horizontalAlignment','center','verticalAlignment','middle')

% Draw axes to plot pacejka's curves
lambda_values = deg2rad(linspace(0,15,100));

ax_fr = axes('Position',[0.6,0.55,0.2*1.2,0.3*1.2],'FontSize',15);
hold all
[~,Fy_fr_values] = pacejka_model(-qa0(2)*795*9.81,kappa_fr,lambda_values,readstruct('./neutral-car.xml').front_tire);
[~,Fy_fr] = pacejka_model(-qa0(2)*795*9.81,kappa_fr,lambda_fr,readstruct('./neutral-car.xml').front_tire);

plot(rad2deg(lambda_values),Fy_fr_values,'-k','LineWidth',2);
plot(-[rad2deg(lambda_fr),rad2deg(lambda_fr)],[0,-Fy_fr],'--k','LineWidth',1)
plot(-rad2deg(lambda_fr), -Fy_fr, 'o','MarkerFaceColor',[1,0,0],'MarkerEdgeColor',[1,0,0]/1.5,'MarkerSize',10,'LineWidth',2);
grid on
box on
xlim([0,15]);
ax_fr.XTick = 0:1:15;
ax_fr.XTickLabel(1:end) = {''};
ax_fr.XTickLabelRotation = 0;

ax_fr.YTick = 0:1000:10000;
ax_fr.YTickLabel(2:2:end) = {''};
ax_fr.YTickLabelRotation = 0;

title('front lateral force, Fy [N]');

ax_rr = axes('Position',[0.6,0.15,0.2*1.2,0.3*1.2],'FontSize',15);
hold all
[~,Fy_rr_values] = pacejka_model(-qa0(4)*795*9.81,kappa_rr,lambda_values,readstruct('./neutral-car.xml').rear_tire);
[~,Fy_rr] = pacejka_model(-qa0(4)*795*9.81,kappa_rr,lambda_rr,readstruct('./neutral-car.xml').rear_tire);

plot(rad2deg(lambda_values),Fy_rr_values,'-k','LineWidth',2);
plot(-[rad2deg(lambda_rr),rad2deg(lambda_rr)],[0,-Fy_rr],'--k','LineWidth',1)
plot(-rad2deg(lambda_rr), -Fy_rr, 'o','MarkerFaceColor',[1,0,0],'MarkerEdgeColor',[1,0,0]/1.5,'MarkerSize',10,'LineWidth',2);
grid on
box on
xlim([0,15]);
ax_rr.XTick = 0:1:15;
ax_rr.XTickLabel(2:2:end) = {''};
ax_rr.XTickLabelRotation = 0;

ax_rr.YTick = 0:1000:10000;
ax_rr.YTickLabel(2:2:end) = {''};
ax_rr.YTickLabelRotation = 0;
xlabel('lateral slip, lambda [deg]');
title('rear lateral force, Fy [N]');


% Draw a car with the velocities
ax_car = axes('Position',[0.35,0.0,0.2,0.8]);
ax_car.Visible = 'off';
hold all
x_center_of_gravity = 1.8;
velocity_reference = (150/3.6)*0.5;
axes(ax_car);

plot_f1(0.73,0.0,90,3.6,0.73*2,skin,[1,1,1],0.2,0.25);

% Draw car velocity
plot([0,0],[-1,5.6],'--k','LineWidth',1)
plot(0,x_center_of_gravity,'o','MarkerFaceColor',[1,0,0],'MarkerEdgeColor',[0.5,0.0,0.0],'MarkerSize',20)
plot([0,q0(6)/velocity_reference*2],[x_center_of_gravity,x_center_of_gravity+q0(5)/velocity_reference*2],'-k','LineWidth',1)
draw_arrow([0,q0(6)/velocity_reference],[x_center_of_gravity,x_center_of_gravity+q0(5)/velocity_reference],[1,0,0])


% Draw rear right velocity
plot([0.73,0.73],[-1.0,3.0],'--k','LineWidth',1)
draw_arrow([0.73,0.73 + v_rr(2)/velocity_reference],[0.0,0.0 + v_rr(1)/velocity_reference],[1,0,0])
axis equal
xlim([-2.5,2.5])
end

function handles = draw_arrow(x,y,color)

head_length = 0.25;
line_width = 4;
handles(1) = plot([x(1),x(1) + (1-head_length)*(x(2)-x(1))],[y(1),y(1) + (1-head_length)*(y(2)-y(1))],'-','LineWidth',line_width,'Color',color);


triangle_base = [x(1),y(1)] + (1-head_length)*[x(2)-x(1),y(2)-y(1)];
dr = [x(2)-x(1),y(2)-y(1)];

triangle_P0 = triangle_base + [dr(2),-dr(1)]*head_length*0.4;
triangle_P1 = triangle_base - [dr(2),-dr(1)]*head_length*0.4;

handles(2) = patch([x(2),triangle_P0(1),triangle_P1(1)], [y(2), triangle_P0(2), triangle_P1(2)],color,'EdgeColor',color);
end

function [Fx,Fy,rho] = pacejka_model(Fz,kappa,lambda,tire_data)
Fz_1 = tire_data.reference_load_1.Text;
Fz_2 = tire_data.reference_load_2.Text;
mu_x_1 = tire_data.mu_x_max_1;
mu_x_2 = tire_data.mu_x_max_2;
kappa_max_1 = tire_data.kappa_max_1;
kappa_max_2 = tire_data.kappa_max_2;
mu_y_1 = tire_data.mu_y_max_1;
mu_y_2 = tire_data.mu_y_max_2;
lambda_max_1 = deg2rad(tire_data.lambda_max_1.Text);
lambda_max_2 = deg2rad(tire_data.lambda_max_2.Text);

Qx = tire_data.Qx;
Qy = tire_data.Qy;
Sx = pi/(2*atan(Qx));
Sy = pi/(2*atan(Qy));
mu_x_max = (Fz-Fz_1)*(mu_x_2-mu_x_1)/(Fz_2-Fz_1) + mu_x_1;
mu_y_max = (Fz-Fz_1)*(mu_y_2-mu_y_1)/(Fz_2-Fz_1) + mu_y_1;
kappa_max = (Fz-Fz_1)*(kappa_max_2-kappa_max_1)/(Fz_2-Fz_1) + kappa_max_1;
lambda_max = (Fz-Fz_1)*(lambda_max_2-lambda_max_1)/(Fz_2-Fz_1) + lambda_max_1;
kappa = kappa.*kappa_max;
rho = sqrt((kappa./kappa_max).^2 + (lambda./lambda_max).^2);
Fx = mu_x_max.*Fz.*sin(Qx.*atan(Sx*rho)).*kappa./(kappa_max.*rho);
Fy = mu_y_max.*Fz.*sin(Qy.*atan(Sy*rho)).*lambda./(lambda_max.*rho);
end

function [r_left, r_right] = create_circular_track(curvature,n_points,width, save_as_xml)

kappa = curvature*ones(1,n_points+1);
theta = linspace(0,2*pi,n_points+1);

x = cos(theta)/curvature;
y = sin(theta)/curvature;


nl = 0.5*width*ones(1,n_points+1);
nr = zeros(1,n_points+1);

x_left = x - nl.*cos(theta);
y_left = y - nl.*sin(theta);

x_right = x + nr.*cos(theta);
y_right = y + nr.*sin(theta);

r_left = [x_left; y_left];
r_right = [x_right; y_right];

dx = diff(x);
dy = diff(y);

darclength = sqrt(dx.*dx + dy.*dy);

arclength = [0,cumsum(darclength)];

track_length = arclength(end);

if save_as_xml
    % Write an XML file
    origin_longitude = 0;
    origin_latittude = 0;
    earth_radius = 6378388;
    reference_latitude = 0;

    docNode = com.mathworks.xml.XMLUtils.createDocument('circular_track');
    toc = docNode.getDocumentElement;
    toc.setAttribute('format','discrete');
    toc.setAttribute('type','closed');

    header_node = docNode.createElement('header');
    toc.appendChild(header_node);

    node = docNode.createElement('track_length');
    node.appendChild(docNode.createTextNode(num2str(track_length)));
    header_node.appendChild(node);

    node = docNode.createElement('L2_error_left');
    node.appendChild(docNode.createTextNode('0.0'));
    header_node.appendChild(node);

    node = docNode.createElement('L2_error_right');
    node.appendChild(docNode.createTextNode('0.0'));
    header_node.appendChild(node);

    node = docNode.createElement('max_error_left');
    node.appendChild(docNode.createTextNode('0.0'));
    header_node.appendChild(node);

    node = docNode.createElement('max_error_right');
    node.appendChild(docNode.createTextNode('0.0'));
    header_node.appendChild(node);

    opt_node = docNode.createElement('optimization');
    toc.appendChild(opt_node);

    node = docNode.createElement('cost_curvature');
    node.appendChild(docNode.createTextNode('0.0'));
    opt_node.appendChild(node);

    node = docNode.createElement('cost_track_limits_smoothness');
    node.appendChild(docNode.createTextNode('0.0'));
    opt_node.appendChild(node);

    node = docNode.createElement('cost_track_limits_errors');
    node.appendChild(docNode.createTextNode('0.0'));
    opt_node.appendChild(node);

    node = docNode.createElement('cost_centerline');
    node.appendChild(docNode.createTextNode('0.0'));
    opt_node.appendChild(node);

    node = docNode.createElement('maximum_kappa');
    node.appendChild(docNode.createTextNode('0.0'));
    opt_node.appendChild(node);

    node = docNode.createElement('maximum_dkappa');
    node.appendChild(docNode.createTextNode('0.0'));
    opt_node.appendChild(node);

    GPS_node = docNode.createElement('GPS_parameters');
    toc.appendChild(GPS_node);


    node = docNode.createElement('origin_longitude');
    node.appendChild(docNode.createTextNode('0.0'));
    GPS_node.appendChild(node);

    node = docNode.createElement('origin_latitude');
    node.appendChild(docNode.createTextNode('0.0'));
    GPS_node.appendChild(node);

    node = docNode.createElement('earth_radius');
    node.appendChild(docNode.createTextNode('6378388'));
    GPS_node.appendChild(node);

    node = docNode.createElement('reference_latitude');
    node.appendChild(docNode.createTextNode('0.0'));
    GPS_node.appendChild(node);

    data_node = docNode.createElement('data');
    data_node.setAttribute('number_of_points',num2str(n_points));
    toc.appendChild(data_node);

    node = docNode.createElement('arclength');
    node.appendChild(docNode.createTextNode(num2str(arclength(1:end-1))));
    data_node.appendChild(node);

    centerline_node = docNode.createElement('centerline');
    data_node.appendChild(centerline_node);

    node = docNode.createElement('x');
    node.appendChild(docNode.createTextNode(num2str(x(1:end-1))));
    centerline_node.appendChild(node);

    node = docNode.createElement('y');
    node.appendChild(docNode.createTextNode(num2str(y(1:end-1))));
    centerline_node.appendChild(node);

    centerline_node = docNode.createElement('left_boundary');
    data_node.appendChild(centerline_node);

    node = docNode.createElement('x');
    node.appendChild(docNode.createTextNode(num2str(x_left(1:end-1))));
    centerline_node.appendChild(node);

    node = docNode.createElement('y');
    node.appendChild(docNode.createTextNode(num2str(y_left(1:end-1))));
    centerline_node.appendChild(node);

    centerline_node = docNode.createElement('right_boundary');
    data_node.appendChild(centerline_node);

    node = docNode.createElement('x');
    node.appendChild(docNode.createTextNode(num2str(x_right(1:end-1))));
    centerline_node.appendChild(node);

    node = docNode.createElement('y');
    node.appendChild(docNode.createTextNode(num2str(y_right(1:end-1))));
    centerline_node.appendChild(node);

    centerline_node = docNode.createElement('left_measured_boundary');
    data_node.appendChild(centerline_node);

    node = docNode.createElement('x');
    node.appendChild(docNode.createTextNode(num2str(x_left(1:end-1))));
    centerline_node.appendChild(node);

    node = docNode.createElement('y');
    node.appendChild(docNode.createTextNode(num2str(y_left(1:end-1))));
    centerline_node.appendChild(node);

    centerline_node = docNode.createElement('right_measured_boundary');
    data_node.appendChild(centerline_node);

    node = docNode.createElement('x');
    node.appendChild(docNode.createTextNode(num2str(x_right(1:end-1))));
    centerline_node.appendChild(node);

    node = docNode.createElement('y');
    node.appendChild(docNode.createTextNode(num2str(y_right(1:end-1))));
    centerline_node.appendChild(node);

    node = docNode.createElement('theta');
    node.appendChild(docNode.createTextNode(num2str(theta(1:end-1)+0.5*pi)));
    data_node.appendChild(node);

    node = docNode.createElement('kappa');
    node.appendChild(docNode.createTextNode(num2str(kappa(1:end-1))));
    data_node.appendChild(node);

    node = docNode.createElement('nl');
    node.appendChild(docNode.createTextNode(num2str(nl(1:end-1))));
    data_node.appendChild(node);

    node = docNode.createElement('nr');
    node.appendChild(docNode.createTextNode(num2str(nr(1:end-1))));
    data_node.appendChild(node);

    node = docNode.createElement('dkappa');
    node.appendChild(docNode.createTextNode(num2str(zeros(1,n_points))));
    data_node.appendChild(node);

    node = docNode.createElement('dnl');
    node.appendChild(docNode.createTextNode(num2str(zeros(1,n_points))));
    data_node.appendChild(node);

    node = docNode.createElement('dnr');
    node.appendChild(docNode.createTextNode(num2str(zeros(1,n_points))));
    data_node.appendChild(node);

    xmlwrite('circular_track.xml', docNode);
end
end

function clean(fastest_lap)

if libisloaded(fastest_lap)
    unloadlibrary(fastest_lap)
end

end