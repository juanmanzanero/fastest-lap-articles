prefix='/home/r660/r660391/storage/';
suffix='.so';

addpath([prefix,'/fastest-lap/src/main/matlab'])

if ( steps_1_to_3 )
    
    % (1) Load library
    if libisloaded('libfastestlapc')
        unloadlibrary libfastestlapc
    end
    
    loadlibrary([prefix,'/fastest-lap/build/lib/libfastestlapc',suffix],[prefix,'/fastest-lap/src/main/c/fastestlapc.h']);
    
    % (2) Construct circuit
    circuit = 'suzuka';
    calllib('libfastestlapc','create_track_from_xml','suzuka',[prefix,'/fastest-lap-projects/2022/Suzuka/0_Preprocessor/suzuka_uniform_2000.xml']);
    
    % (2.1) Get track coordinates
    track_length = calllib("libfastestlapc", "track_download_length", circuit);
    N = calllib("libfastestlapc","track_download_number_of_points", circuit);
    s = calllib("libfastestlapc","track_download_data",zeros(1,N),circuit, N, 'arclength');
    x_center = calllib("libfastestlapc","track_download_data",zeros(1,N),circuit,N, 'centerline.x');
    y_center = calllib("libfastestlapc","track_download_data",zeros(1,N),circuit,N, 'centerline.y');
    x_left = calllib("libfastestlapc","track_download_data",zeros(1,N),circuit,N, 'left.x');
    y_left = calllib("libfastestlapc","track_download_data",zeros(1,N),circuit,N, 'left.y');
    x_right = calllib("libfastestlapc","track_download_data",zeros(1,N),circuit,N, 'right.x');
    y_right = calllib("libfastestlapc","track_download_data",zeros(1,N),circuit,N, 'right.y');
    
    % (3) Construct car
    vehicle = 'vehicle';
    calllib("libfastestlapc","create_vehicle_from_xml",vehicle,'alonso_qualifying_suzuka_2022.xml');
    vehicle_data = readstruct('alonso_qualifying_suzuka_2022.xml');

    % (3.1) Add variation of the aerodynamic coefficients while using the
    % DRS
    i_drs_start = 1450;
    i_drs_end   = 1820;
    cd = 0.99642659990346749;
    cl = 2.7101908445246665;
    x_press = - 0.092900898328350232;

    cd_drs = 0.744011;
    cl_drs = 2.288312;
    x_press_drs = 0.262680;
    drs_mesh = [0.0,s(i_drs_start)-10,s(i_drs_start), s(i_drs_end)-10, s(i_drs_end), 7000.0];
    drs_vars = [  0,                0,             1,               1,            0,      0]; 
    calllib("libfastestlapc", "vehicle_declare_new_variable_parameter", vehicle, 'vehicle/chassis/aerodynamics/cd', 'cd;cd_drs',  2, [cd,cd_drs], numel(drs_mesh), drs_vars, drs_mesh);
    calllib("libfastestlapc", "vehicle_declare_new_variable_parameter", vehicle, 'vehicle/chassis/aerodynamics/cl', 'cl;cl_drs',  2, [cl,cl_drs], numel(drs_mesh), drs_vars, drs_mesh);
    calllib("libfastestlapc", "vehicle_declare_new_variable_parameter", vehicle, 'vehicle/chassis/pressure_center/x', 'x_press;x_press_drs',  2, [x_press,x_press_drs], numel(drs_mesh), drs_vars, drs_mesh);

    
    % (3) Simulation
    options = '<options>';
    options = [options,    '<control_variables>'];
    options = [options,    '    <chassis.brake-bias optimal_control_type="hypermesh">'];
    options = [options,    '        <hypermesh>0.0 2057.8639 2712.7711 3064.9656 3399.696 4628.0108 5230.5254</hypermesh>'];
    options = [options,    '    </chassis.brake-bias>'];
    options = [options,    '</control_variables>'];
    options = [options,    '<output_variables>'];
    options = [options,    '    <prefix> run/ </prefix>'];
    options = [options,    '</output_variables>'];
    options = [options,'</options>'];
    calllib("libfastestlapc","optimal_laptime",vehicle,circuit,length(s),s,options);
    
    % (3.1) Download the variables
    run.laptime = calllib("libfastestlapc","download_scalar", 'run/laptime');
    run.x=zeros(1,length(s));        run.x        = calllib("libfastestlapc","download_vector",run.x, length(s), 'run/chassis.position.x');
    run.y=zeros(1,length(s));        run.y        = calllib("libfastestlapc","download_vector",run.y, length(s), 'run/chassis.position.y');
    run.kappa_fl=zeros(1,length(s)); run.kappa_fl = calllib("libfastestlapc","download_vector",run.kappa_fl, length(s), 'run/front-axle.left-tire.kappa');
    run.kappa_fr=zeros(1,length(s)); run.kappa_fr = calllib("libfastestlapc","download_vector",run.kappa_fr, length(s), 'run/front-axle.right-tire.kappa');
    run.kappa_rl=zeros(1,length(s)); run.kappa_rl = calllib("libfastestlapc","download_vector",run.kappa_rl, length(s), 'run/rear-axle.left-tire.kappa');
    run.kappa_rr=zeros(1,length(s)); run.kappa_rr = calllib("libfastestlapc","download_vector",run.kappa_rr, length(s), 'run/rear-axle.right-tire.kappa');
    run.u=zeros(1,length(s));        run.u        = calllib("libfastestlapc","download_vector",run.u, length(s), 'run/chassis.velocity.x');
    run.v=zeros(1,length(s));        run.v        = calllib("libfastestlapc","download_vector",run.v, length(s), 'run/chassis.velocity.y');
    run.omega=zeros(1,length(s));    run.omega    = calllib("libfastestlapc","download_vector",run.omega, length(s), 'run/chassis.omega.z');
    run.time=zeros(1,length(s));     run.time     = calllib("libfastestlapc","download_vector",run.time, length(s), 'run/time');
    run.n=zeros(1,length(s));        run.n        = calllib("libfastestlapc","download_vector",run.n, length(s), 'run/road.lateral-displacement');
    run.alpha=zeros(1,length(s));    run.alpha    = calllib("libfastestlapc","download_vector",run.alpha, length(s), 'run/road.track-heading-angle');
    run.delta=zeros(1,length(s));    run.delta    = calllib("libfastestlapc","download_vector",run.delta, length(s), 'run/front-axle.steering-angle');
    run.throttle=zeros(1,length(s)); run.throttle = calllib("libfastestlapc","download_vector",run.throttle, length(s), 'run/chassis.throttle');
    run.brake_bias=zeros(1,length(s)); run.brake_bias = calllib("libfastestlapc","download_vector",run.brake_bias, length(s), 'run/chassis.brake-bias');
    run.Fz_fl=zeros(1,length(s));    run.Fz_fl    = calllib("libfastestlapc","download_vector",run.Fz_fl, length(s), 'run/chassis.Fz_fl');
    run.Fz_fr=zeros(1,length(s));    run.Fz_fr    = calllib("libfastestlapc","download_vector",run.Fz_fr, length(s), 'run/chassis.Fz_fr');
    run.Fz_rl=zeros(1,length(s));    run.Fz_rl    = calllib("libfastestlapc","download_vector",run.Fz_rl, length(s), 'run/chassis.Fz_rl');
    run.Fz_rr=zeros(1,length(s));    run.Fz_rr    = calllib("libfastestlapc","download_vector",run.Fz_rr, length(s), 'run/chassis.Fz_rr');
    
    dissipation_fl = calllib("libfastestlapc","download_vector",zeros(1,length(s)), length(s), 'run/front-axle.left-tire.dissipation');
    dissipation_fr = calllib("libfastestlapc","download_vector",zeros(1,length(s)), length(s), 'run/front-axle.right-tire.dissipation');
    dissipation_rl = calllib("libfastestlapc","download_vector",zeros(1,length(s)), length(s), 'run/rear-axle.left-tire.dissipation');
    dissipation_rr = calllib("libfastestlapc","download_vector",zeros(1,length(s)), length(s), 'run/rear-axle.right-tire.dissipation');
    
    energy_fl = zeros(1,length(s));
    energy_fr = zeros(1,length(s));
    energy_rl = zeros(1,length(s));
    energy_rr = zeros(1,length(s));
    for i = 2 : length(s)
        energy_fl(i) = energy_fl(i-1) + 0.5*(run.time(i)-run.time(i-1))*(dissipation_fl(i-1) + dissipation_fl(i));
        energy_fr(i) = energy_fr(i-1) + 0.5*(run.time(i)-run.time(i-1))*(dissipation_fr(i-1) + dissipation_fr(i));
        energy_rl(i) = energy_rl(i-1) + 0.5*(run.time(i)-run.time(i-1))*(dissipation_rl(i-1) + dissipation_rl(i));
        energy_rr(i) = energy_rr(i-1) + 0.5*(run.time(i)-run.time(i-1))*(dissipation_rr(i-1) + dissipation_rr(i));
    end
    
    % (3.2) Put the variables into the states and controls vectors
    q = [run.kappa_fl; run.kappa_fr;  run.kappa_rl; run.kappa_rr;...
        run.u; run.v; run.omega; run.time; run.n; run.alpha];
    qa = [run.Fz_fl; run.Fz_fr; run.Fz_rl; run.Fz_rr];
    u  = [run.delta; run.throttle; run.brake_bias];
    energy = [energy_fl; energy_fr; energy_rl; energy_rr];
    
    r_center = [x_center ; y_center];
    
    s_track = [s,track_length];
    x_center = [x_center, x_center(1)];
    y_center = [y_center, y_center(1)];
    x_left = [x_left, x_left(1)];
    y_left = [y_left, y_left(1)];
    x_right = [x_right, x_right(1)];
    y_right = [y_right, y_right(1)];
    
    
    % (3.2.1) Make periodic: add part of another lap
    n_extra = 1600;
    n_extra_prev = 1600;
    q = [q(:,N-n_extra_prev+1:end), q, q(:,1:n_extra)];
    x = [run.x(:,N-n_extra_prev+1:end), run.x, run.x(:,1:n_extra)];
    y = [run.y(:,N-n_extra_prev+1:end), run.y, run.y(:,1:n_extra)];
    qa = [qa(:,N-n_extra_prev+1:end), qa, qa(:,1:n_extra)];
    u = [u(:,N-n_extra_prev+1:end), u, u(:,1:n_extra)];
    energy = [energy(:,N-n_extra_prev+1:end), energy, energy(:,1:n_extra)];
    r_center = [r_center(:,N-n_extra_prev+1:end), r_center, r_center(:,1:n_extra)];
    s = [s(N-n_extra_prev+1:end)-track_length, s, s(1:n_extra)+track_length];
    q(8,n_extra_prev+1+N:end) = q(8,n_extra_prev+1+N:end)+run.laptime;
    q(8,1:n_extra_prev) = q(8,1:n_extra_prev)-run.laptime;
    
    
    
    
    % (4) Interpolation: we want 30FPS, so the time grid will be with dt = 1/30
    
    % (4.1) Interpolate the track to 4000 points
    n_track = 4000;
    x_center_plot = interp1(s_track,x_center,linspace(s_track(1),s_track(end),n_track),'spline');
    y_center_plot = interp1(s_track,y_center,linspace(s_track(1),s_track(end),n_track),'spline');
    x_left_plot = interp1(s_track,x_left,linspace(s_track(1),s_track(end),n_track),'spline');
    y_left_plot = interp1(s_track,y_left,linspace(s_track(1),s_track(end),n_track),'spline');
    x_right_plot = interp1(s_track,x_right,linspace(s_track(1),s_track(end),n_track),'spline');
    y_right_plot = interp1(s_track,y_right,linspace(s_track(1),s_track(end),n_track),'spline');
    
    % (4.2) Interpolate the solution to the points given by the FPS
    dt = 1/30;
    t_plot = -5:dt:run.laptime+5;
    
    s_plot = interp1(q(8,:), s, t_plot,'spline')';
    q_plot = interp1(q(8,:), q', t_plot,'spline')';
    qa_plot = interp1(q(8,:), qa', t_plot,'spline')';
    u_plot = interp1(q(8,:), u', t_plot,'spline')';
    energy_plot = interp1(q(8,:), energy', t_plot,'spline')';
    x_plot = interp1(q(8,:), x, t_plot,'spline')';
    y_plot = interp1(q(8,:), y, t_plot,'spline')';
    r_center_plot = interp1(q(8,:), r_center', t_plot,'spline')';
    
    % (4.2.1) Correct s so that is inside [0,track_length]
    s_plot(s_plot < 0) = s_plot(s_plot < 0) + track_length;
    s_plot(s_plot > track_length) = s_plot(s_plot > track_length) - track_length;

    save('run.mat');

else

    
    % (1) Load library
    if libisloaded('libfastestlapc')
        unloadlibrary libfastestlapc
    end
    
    loadlibrary([prefix,'/fastest-lap/build/lib/libfastestlapc', suffix],[prefix,'/fastest-lap/src/main/c/fastestlapc.h']);

    prefix_save = prefix;
    load('run.mat');
    
    prefix = prefix_save;

    % (2) Construct track
    circuit = 'suzuka';
    calllib('libfastestlapc','create_track_from_xml','suzuka',[prefix,'/fastest-lap-projects/2022/Suzuka/0_Preprocessor/suzuka_uniform_2000.xml']);

    %(3) Construct vehicle and link track
    vehicle = 'vehicle';
    calllib("libfastestlapc","create_vehicle_from_xml",vehicle,'alonso_qualifying_suzuka_2022.xml');
    calllib("libfastestlapc","vehicle_change_track",vehicle,circuit); 
end

length(t_plot)
close all
for i = i_start:i_end
    h=plot_run_dashboard(i,t_plot,s_plot,x_plot,y_plot,r_center_plot,q_plot,qa_plot,u_plot, x_center_plot, y_center_plot, x_left_plot, y_left_plot,...
        x_right_plot, y_right_plot, run.laptime, vehicle, vehicle_data,-min(min(qa)),energy_plot, 'alpine');
    print(['figs/fig_',num2str(i)],'-dpng');
    close(h);
end
