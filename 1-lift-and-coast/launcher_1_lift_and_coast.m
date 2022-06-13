if (~exist('restart','var'))
    restart = true;
end

set(groot,'defaultAxesFontName','Formula1');

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
    
    % (2) Construct track
    track = 'catalunya';
    calllib('libfastestlapc','create_track',track,[prefix,'/database/tracks/catalunya_2022/catalunya_2022_adapted.xml'],fastestlap_default_options('create_track'));
    
    % (2.1) Get the arclength
    N = calllib("libfastestlapc","download_vector_table_variable_size",'track/s');
    s = calllib("libfastestlapc","download_vector_table_variable",zeros(1,N), N, 'track/s');
    track_length = 4669.8382163304168;
    
    % (2.1) Get track coordinates
    [x_center,y_center,x_left,y_left,x_right,y_right,theta] = calllib('libfastestlapc','track_coordinates',zeros(1,N),zeros(1,N),zeros(1,N),zeros(1,N),zeros(1,N),zeros(1,N),zeros(1,N),track,N,s);
    track_data.x_center = x_center(1:N);
    track_data.y_center = y_center(1:N);
    track_data.x_left = x_left(1:N);
    track_data.y_left = y_left(1:N);
    track_data.x_right = x_right(1:N);
    track_data.y_right = y_right(1:N);
    
    % (3) Construct car
    vehicle = 'car';
    calllib("libfastestlapc","create_vehicle",vehicle,'limebeer-2014-f1','ferrari-2022-catalunya.xml');
    calllib("libfastestlapc","set_scalar_parameter",vehicle,'vehicle/rear-axle/smooth_throttle_coeff',1.0e-2);
    drs_mesh = [0.0,90.0,100.0,880.0,890.0,3190.0,3200.0,3540.0,3550.0,5000.0];
    drs_vars = [0,     0,    1,    1,    0,     0,     1,     1,     0,     0];
    calllib("libfastestlapc", "vehicle_declare_new_variable_parameter", vehicle, 'vehicle/chassis/aerodynamics/cd', 'cd;cd_drs',  2, [1.2999951232589213,0.8914], numel(drs_mesh), drs_vars, drs_mesh);
    % (4) Compute optimal laptime
    
    % (4.1) Write options
    options =          '<options>';
    options = [options,    '<save_variables>'];
    options = [options,    '    <prefix>run/</prefix>'];
    options = [options,    '    <variables>'];
    options = [options,    '        <laptime/>'];
    options = [options,    '        <x/>'];
    options = [options,    '        <y/>'];
    options = [options,    '        <front_axle.left_tire.kappa/>'];
    options = [options,    '        <front_axle.right_tire.kappa/>'];
    options = [options,    '        <rear_axle.left_tire.kappa/>'];
    options = [options,    '        <rear_axle.right_tire.kappa/>'];
    options = [options,    '        <front_axle.left_tire.dissipation/>'];
    options = [options,    '        <front_axle.right_tire.dissipation/>'];
    options = [options,    '        <rear_axle.left_tire.dissipation/>'];
    options = [options,    '        <rear_axle.right_tire.dissipation/>'];
    options = [options,    '        <u/>'];
    options = [options,    '        <v/>'];
    options = [options,    '        <omega/>'];
    options = [options,    '        <time/>'];
    options = [options,    '        <n/>'];
    options = [options,    '        <alpha/>'];
    options = [options,    '        <delta/>'];
    options = [options,    '        <throttle/>'];
    options = [options,    '        <brake-bias/>'];
    options = [options,    '        <Fz_fl/>'];
    options = [options,    '        <Fz_fr/>'];
    options = [options,    '        <Fz_rl/>'];
    options = [options,    '        <Fz_rr/>'];
    options = [options,    '        <integral_quantities.engine-energy/>'];
    options = [options,    '    </variables>'];
    options = [options,    '</save_variables>'];
    options = [options,'</options>'];
    
    % (4.2) Compute
    fprintf('Running first optimization...');
    calllib("libfastestlapc","optimal_laptime",vehicle,track,length(s),s,options);
    fprintf(' [DONE]\n');
    
    % (4.3) Download result
    run_1.laptime = calllib("libfastestlapc","download_scalar_table_variable", 'run/laptime');
    run_1.x=zeros(1,length(s));        run_1.x        = calllib("libfastestlapc","download_vector_table_variable",run_1.x, length(s), 'run/x');
    run_1.y=zeros(1,length(s));        run_1.y        = calllib("libfastestlapc","download_vector_table_variable",run_1.y, length(s), 'run/y');
    run_1.kappa_fl=zeros(1,length(s)); run_1.kappa_fl = calllib("libfastestlapc","download_vector_table_variable",run_1.kappa_fl, length(s), 'run/front_axle.left_tire.kappa');
    run_1.kappa_fr=zeros(1,length(s)); run_1.kappa_fr = calllib("libfastestlapc","download_vector_table_variable",run_1.kappa_fr, length(s), 'run/front_axle.right_tire.kappa');
    run_1.kappa_rl=zeros(1,length(s)); run_1.kappa_rl = calllib("libfastestlapc","download_vector_table_variable",run_1.kappa_rl, length(s), 'run/rear_axle.left_tire.kappa');
    run_1.kappa_rr=zeros(1,length(s)); run_1.kappa_rr = calllib("libfastestlapc","download_vector_table_variable",run_1.kappa_rr, length(s), 'run/rear_axle.right_tire.kappa');
    run_1.u=zeros(1,length(s));        run_1.u        = calllib("libfastestlapc","download_vector_table_variable",run_1.u, length(s), 'run/u');
    run_1.v=zeros(1,length(s));        run_1.v        = calllib("libfastestlapc","download_vector_table_variable",run_1.v, length(s), 'run/v');
    run_1.omega=zeros(1,length(s));    run_1.omega    = calllib("libfastestlapc","download_vector_table_variable",run_1.omega, length(s), 'run/omega');
    run_1.time=zeros(1,length(s));     run_1.time     = calllib("libfastestlapc","download_vector_table_variable",run_1.time, length(s), 'run/time');
    run_1.n=zeros(1,length(s));        run_1.n        = calllib("libfastestlapc","download_vector_table_variable",run_1.n, length(s), 'run/n');
    run_1.alpha=zeros(1,length(s));    run_1.alpha    = calllib("libfastestlapc","download_vector_table_variable",run_1.alpha, length(s), 'run/alpha');
    run_1.delta=zeros(1,length(s));    run_1.delta    = calllib("libfastestlapc","download_vector_table_variable",run_1.delta, length(s), 'run/delta');
    run_1.throttle=zeros(1,length(s)); run_1.throttle = calllib("libfastestlapc","download_vector_table_variable",run_1.throttle, length(s), 'run/throttle');
    run_1.brake_bias=zeros(1,length(s)); run_1.brake_bias = calllib("libfastestlapc","download_vector_table_variable",run_1.brake_bias, length(s), 'run/brake-bias');
    run_1.Fz_fl=zeros(1,length(s));    run_1.Fz_fl    = calllib("libfastestlapc","download_vector_table_variable",run_1.Fz_fl, length(s), 'run/Fz_fl');
    run_1.Fz_fr=zeros(1,length(s));    run_1.Fz_fr    = calllib("libfastestlapc","download_vector_table_variable",run_1.Fz_fr, length(s), 'run/Fz_fr');
    run_1.Fz_rl=zeros(1,length(s));    run_1.Fz_rl    = calllib("libfastestlapc","download_vector_table_variable",run_1.Fz_rl, length(s), 'run/Fz_rl');
    run_1.Fz_rr=zeros(1,length(s));    run_1.Fz_rr    = calllib("libfastestlapc","download_vector_table_variable",run_1.Fz_rr, length(s), 'run/Fz_rr');
    run_1.dissipation_fl = calllib("libfastestlapc","download_vector_table_variable",zeros(1,length(s)), length(s), 'run/front_axle.left_tire.dissipation');
    run_1.dissipation_fr = calllib("libfastestlapc","download_vector_table_variable",zeros(1,length(s)), length(s), 'run/front_axle.right_tire.dissipation');
    run_1.dissipation_rl = calllib("libfastestlapc","download_vector_table_variable",zeros(1,length(s)), length(s), 'run/rear_axle.left_tire.dissipation');
    run_1.dissipation_rr = calllib("libfastestlapc","download_vector_table_variable",zeros(1,length(s)), length(s), 'run/rear_axle.right_tire.dissipation');
    engine_energy = calllib("libfastestlapc","download_scalar_table_variable",'run/integral_quantities.engine-energy');
    run_1.energy_fl = zeros(1,length(s));
    run_1.energy_fr = zeros(1,length(s));
    run_1.energy_rl = zeros(1,length(s));
    run_1.energy_rr = zeros(1,length(s));
    
    for i = 2 : length(s)
        run_1.energy_fl(i) = run_1.energy_fl(i-1) + 0.5*(run_1.time(i)-run_1.time(i-1))*(run_1.dissipation_fl(i-1) + run_1.dissipation_fl(i));
        run_1.energy_fr(i) = run_1.energy_fr(i-1) + 0.5*(run_1.time(i)-run_1.time(i-1))*(run_1.dissipation_fr(i-1) + run_1.dissipation_fr(i));
        run_1.energy_rl(i) = run_1.energy_rl(i-1) + 0.5*(run_1.time(i)-run_1.time(i-1))*(run_1.dissipation_rl(i-1) + run_1.dissipation_rl(i));
        run_1.energy_rr(i) = run_1.energy_rr(i-1) + 0.5*(run_1.time(i)-run_1.time(i-1))*(run_1.dissipation_rr(i-1) + run_1.dissipation_rr(i));
    end
    
    run_1.q = [run_1.kappa_fl; run_1.kappa_fr;  run_1.kappa_rl; run_1.kappa_rr;...
             run_1.u; run_1.v; run_1.omega; run_1.time; run_1.n; run_1.alpha];
    run_1.qa = [run_1.Fz_fl; run_1.Fz_fr; run_1.Fz_rl; run_1.Fz_rr];
    run_1.control_variables  = [run_1.delta; run_1.throttle; run_1.brake_bias];
    run_1.energy = [run_1.energy_fl; run_1.energy_fr; run_1.energy_rl; run_1.energy_rr];
    run_1.vehicle = vehicle;
    run_1.engine_power = 595.39174549309666;
    calllib("libfastestlapc","clear_tables_by_prefix",'run/');
    
    % (5) Compute optimal laptime with energy limits
    
    % (5.1) Write options
    options =          '<options>';
    options = [options,    '<save_variables>'];
    options = [options,    '    <prefix>run/</prefix>'];
    options = [options,    '    <variables>'];
    options = [options,    '        <laptime/>'];
    options = [options,    '        <x/>'];
    options = [options,    '        <y/>'];
    options = [options,    '        <front_axle.left_tire.kappa/>'];
    options = [options,    '        <front_axle.right_tire.kappa/>'];
    options = [options,    '        <rear_axle.left_tire.kappa/>'];
    options = [options,    '        <rear_axle.right_tire.kappa/>'];
    options = [options,    '        <front_axle.left_tire.dissipation/>'];
    options = [options,    '        <front_axle.right_tire.dissipation/>'];
    options = [options,    '        <rear_axle.left_tire.dissipation/>'];
    options = [options,    '        <rear_axle.right_tire.dissipation/>'];
    options = [options,    '        <u/>'];
    options = [options,    '        <v/>'];
    options = [options,    '        <omega/>'];
    options = [options,    '        <time/>'];
    options = [options,    '        <n/>'];
    options = [options,    '        <alpha/>'];
    options = [options,    '        <delta/>'];
    options = [options,    '        <throttle/>'];
    options = [options,    '        <brake-bias/>'];
    options = [options,    '        <Fz_fl/>'];
    options = [options,    '        <Fz_fr/>'];
    options = [options,    '        <Fz_rl/>'];
    options = [options,    '        <Fz_rr/>'];
    options = [options,    '    </variables>'];
    options = [options,    '</save_variables>'];
    options = [options,    '<integral_constraints>'];
    options = [options,        '<engine-energy>'];
    options = [options,            '<lower_bound> 0.0 </lower_bound>'];
    options = [options,            '<upper_bound>',num2str(0.9*engine_energy),'</upper_bound>'];
    options = [options,        '</engine-energy>'];
    options = [options,    '</integral_constraints>'];
    options = [options,'</options>'];
    
    % (5.2) Compute
    fprintf('Running second optimization...');
    calllib("libfastestlapc","optimal_laptime",vehicle,track,length(s),s,options);
    fprintf(' [DONE]\n');
    
    % (5.3) Download result
    run_2.laptime = calllib("libfastestlapc","download_scalar_table_variable", 'run/laptime');
    run_2.x=zeros(1,length(s));        run_2.x        = calllib("libfastestlapc","download_vector_table_variable",run_2.x, length(s), 'run/x');
    run_2.y=zeros(1,length(s));        run_2.y        = calllib("libfastestlapc","download_vector_table_variable",run_2.y, length(s), 'run/y');
    run_2.kappa_fl=zeros(1,length(s)); run_2.kappa_fl = calllib("libfastestlapc","download_vector_table_variable",run_2.kappa_fl, length(s), 'run/front_axle.left_tire.kappa');
    run_2.kappa_fr=zeros(1,length(s)); run_2.kappa_fr = calllib("libfastestlapc","download_vector_table_variable",run_2.kappa_fr, length(s), 'run/front_axle.right_tire.kappa');
    run_2.kappa_rl=zeros(1,length(s)); run_2.kappa_rl = calllib("libfastestlapc","download_vector_table_variable",run_2.kappa_rl, length(s), 'run/rear_axle.left_tire.kappa');
    run_2.kappa_rr=zeros(1,length(s)); run_2.kappa_rr = calllib("libfastestlapc","download_vector_table_variable",run_2.kappa_rr, length(s), 'run/rear_axle.right_tire.kappa');
    run_2.u=zeros(1,length(s));        run_2.u        = calllib("libfastestlapc","download_vector_table_variable",run_2.u, length(s), 'run/u');
    run_2.v=zeros(1,length(s));        run_2.v        = calllib("libfastestlapc","download_vector_table_variable",run_2.v, length(s), 'run/v');
    run_2.omega=zeros(1,length(s));    run_2.omega    = calllib("libfastestlapc","download_vector_table_variable",run_2.omega, length(s), 'run/omega');
    run_2.time=zeros(1,length(s));     run_2.time     = calllib("libfastestlapc","download_vector_table_variable",run_2.time, length(s), 'run/time');
    run_2.n=zeros(1,length(s));        run_2.n        = calllib("libfastestlapc","download_vector_table_variable",run_2.n, length(s), 'run/n');
    run_2.alpha=zeros(1,length(s));    run_2.alpha    = calllib("libfastestlapc","download_vector_table_variable",run_2.alpha, length(s), 'run/alpha');
    run_2.delta=zeros(1,length(s));    run_2.delta    = calllib("libfastestlapc","download_vector_table_variable",run_2.delta, length(s), 'run/delta');
    run_2.throttle=zeros(1,length(s)); run_2.throttle = calllib("libfastestlapc","download_vector_table_variable",run_2.throttle, length(s), 'run/throttle');
    run_2.brake_bias=zeros(1,length(s)); run_2.brake_bias = calllib("libfastestlapc","download_vector_table_variable",run_2.brake_bias, length(s), 'run/brake-bias');
    run_2.Fz_fl=zeros(1,length(s));    run_2.Fz_fl    = calllib("libfastestlapc","download_vector_table_variable",run_2.Fz_fl, length(s), 'run/Fz_fl');
    run_2.Fz_fr=zeros(1,length(s));    run_2.Fz_fr    = calllib("libfastestlapc","download_vector_table_variable",run_2.Fz_fr, length(s), 'run/Fz_fr');
    run_2.Fz_rl=zeros(1,length(s));    run_2.Fz_rl    = calllib("libfastestlapc","download_vector_table_variable",run_2.Fz_rl, length(s), 'run/Fz_rl');
    run_2.Fz_rr=zeros(1,length(s));    run_2.Fz_rr    = calllib("libfastestlapc","download_vector_table_variable",run_2.Fz_rr, length(s), 'run/Fz_rr');
    run_2.dissipation_fl = calllib("libfastestlapc","download_vector_table_variable",zeros(1,length(s)), length(s), 'run/front_axle.left_tire.dissipation');
    run_2.dissipation_fr = calllib("libfastestlapc","download_vector_table_variable",zeros(1,length(s)), length(s), 'run/front_axle.right_tire.dissipation');
    run_2.dissipation_rl = calllib("libfastestlapc","download_vector_table_variable",zeros(1,length(s)), length(s), 'run/rear_axle.left_tire.dissipation');
    run_2.dissipation_rr = calllib("libfastestlapc","download_vector_table_variable",zeros(1,length(s)), length(s), 'run/rear_axle.right_tire.dissipation');
    
    run_2.energy_fl = zeros(1,length(s));
    run_2.energy_fr = zeros(1,length(s));
    run_2.energy_rl = zeros(1,length(s));
    run_2.energy_rr = zeros(1,length(s));
    
    for i = 2 : length(s)
        run_2.energy_fl(i) = run_2.energy_fl(i-1) + 0.5*(run_2.time(i)-run_2.time(i-1))*(run_2.dissipation_fl(i-1) + run_2.dissipation_fl(i));
        run_2.energy_fr(i) = run_2.energy_fr(i-1) + 0.5*(run_2.time(i)-run_2.time(i-1))*(run_2.dissipation_fr(i-1) + run_2.dissipation_fr(i));
        run_2.energy_rl(i) = run_2.energy_rl(i-1) + 0.5*(run_2.time(i)-run_2.time(i-1))*(run_2.dissipation_rl(i-1) + run_2.dissipation_rl(i));
        run_2.energy_rr(i) = run_2.energy_rr(i-1) + 0.5*(run_2.time(i)-run_2.time(i-1))*(run_2.dissipation_rr(i-1) + run_2.dissipation_rr(i));
    end
    
    run_2.q = [run_2.kappa_fl; run_2.kappa_fr;  run_2.kappa_rl; run_2.kappa_rr;...
             run_2.u; run_2.v; run_2.omega; run_2.time; run_2.n; run_2.alpha];
    run_2.qa = [run_2.Fz_fl; run_2.Fz_fr; run_2.Fz_rl; run_2.Fz_rr];
    run_2.control_variables  = [run_2.delta; run_2.throttle; run_2.brake_bias];
    run_2.energy = [run_2.energy_fl; run_2.energy_fr; run_2.energy_rl; run_2.energy_rr];
    run_2.vehicle = vehicle;
    run_2.engine_power = 595.39174549309666;    
    
    
    % (6) Compute optimal laptime with turn down engine
    
    % (6.1) Write options
    options =          '<options>';
    options = [options,    '<save_variables>'];
    options = [options,    '    <prefix>run_3/</prefix>'];
    options = [options,    '    <variables>'];
    options = [options,    '        <laptime/>'];
    options = [options,    '        <x/>'];
    options = [options,    '        <y/>'];
    options = [options,    '        <front_axle.left_tire.kappa/>'];
    options = [options,    '        <front_axle.right_tire.kappa/>'];
    options = [options,    '        <rear_axle.left_tire.kappa/>'];
    options = [options,    '        <rear_axle.right_tire.kappa/>'];
    options = [options,    '        <front_axle.left_tire.dissipation/>'];
    options = [options,    '        <front_axle.right_tire.dissipation/>'];
    options = [options,    '        <rear_axle.left_tire.dissipation/>'];
    options = [options,    '        <rear_axle.right_tire.dissipation/>'];
    options = [options,    '        <u/>'];
    options = [options,    '        <v/>'];
    options = [options,    '        <omega/>'];
    options = [options,    '        <time/>'];
    options = [options,    '        <n/>'];
    options = [options,    '        <alpha/>'];
    options = [options,    '        <delta/>'];
    options = [options,    '        <throttle/>'];
    options = [options,    '        <brake-bias/>'];
    options = [options,    '        <Fz_fl/>'];
    options = [options,    '        <Fz_fr/>'];
    options = [options,    '        <Fz_rl/>'];
    options = [options,    '        <Fz_rr/>'];
    options = [options,    '        <integral_quantities.engine-energy/>'];
    options = [options,    '    </variables>'];
    options = [options,    '</save_variables>'];
    options = [options,'</options>'];
    
    % (6.2) Compute
    calllib("libfastestlapc","set_scalar_parameter",vehicle,'vehicle/chassis/maximum_throttle',0.8375);
    fprintf('Running third optimization...');
    calllib("libfastestlapc","optimal_laptime",vehicle,track,length(s),s,options);
    fprintf(' [DONE]\n');
    calllib("libfastestlapc","set_scalar_parameter",vehicle,'vehicle/chassis/maximum_throttle',1.0);
    
    % (6.3) Download result
    run_3.laptime = calllib("libfastestlapc","download_scalar_table_variable", 'run_3/laptime');
    run_3.x=zeros(1,length(s));        run_3.x        = calllib("libfastestlapc","download_vector_table_variable",run_3.x, length(s), 'run_3/x');
    run_3.y=zeros(1,length(s));        run_3.y        = calllib("libfastestlapc","download_vector_table_variable",run_3.y, length(s), 'run_3/y');
    run_3.kappa_fl=zeros(1,length(s)); run_3.kappa_fl = calllib("libfastestlapc","download_vector_table_variable",run_3.kappa_fl, length(s), 'run_3/front_axle.left_tire.kappa');
    run_3.kappa_fr=zeros(1,length(s)); run_3.kappa_fr = calllib("libfastestlapc","download_vector_table_variable",run_3.kappa_fr, length(s), 'run_3/front_axle.right_tire.kappa');
    run_3.kappa_rl=zeros(1,length(s)); run_3.kappa_rl = calllib("libfastestlapc","download_vector_table_variable",run_3.kappa_rl, length(s), 'run_3/rear_axle.left_tire.kappa');
    run_3.kappa_rr=zeros(1,length(s)); run_3.kappa_rr = calllib("libfastestlapc","download_vector_table_variable",run_3.kappa_rr, length(s), 'run_3/rear_axle.right_tire.kappa');
    run_3.u=zeros(1,length(s));        run_3.u        = calllib("libfastestlapc","download_vector_table_variable",run_3.u, length(s), 'run_3/u');
    run_3.v=zeros(1,length(s));        run_3.v        = calllib("libfastestlapc","download_vector_table_variable",run_3.v, length(s), 'run_3/v');
    run_3.omega=zeros(1,length(s));    run_3.omega    = calllib("libfastestlapc","download_vector_table_variable",run_3.omega, length(s), 'run_3/omega');
    run_3.time=zeros(1,length(s));     run_3.time     = calllib("libfastestlapc","download_vector_table_variable",run_3.time, length(s), 'run_3/time');
    run_3.n=zeros(1,length(s));        run_3.n        = calllib("libfastestlapc","download_vector_table_variable",run_3.n, length(s), 'run_3/n');
    run_3.alpha=zeros(1,length(s));    run_3.alpha    = calllib("libfastestlapc","download_vector_table_variable",run_3.alpha, length(s), 'run_3/alpha');
    run_3.delta=zeros(1,length(s));    run_3.delta    = calllib("libfastestlapc","download_vector_table_variable",run_3.delta, length(s), 'run_3/delta');
    run_3.throttle=zeros(1,length(s)); run_3.throttle = calllib("libfastestlapc","download_vector_table_variable",run_3.throttle, length(s), 'run_3/throttle');
    run_3.brake_bias=zeros(1,length(s)); run_3.brake_bias = calllib("libfastestlapc","download_vector_table_variable",run_3.brake_bias, length(s), 'run_3/brake-bias');
    run_3.Fz_fl=zeros(1,length(s));    run_3.Fz_fl    = calllib("libfastestlapc","download_vector_table_variable",run_3.Fz_fl, length(s), 'run_3/Fz_fl');
    run_3.Fz_fr=zeros(1,length(s));    run_3.Fz_fr    = calllib("libfastestlapc","download_vector_table_variable",run_3.Fz_fr, length(s), 'run_3/Fz_fr');
    run_3.Fz_rl=zeros(1,length(s));    run_3.Fz_rl    = calllib("libfastestlapc","download_vector_table_variable",run_3.Fz_rl, length(s), 'run_3/Fz_rl');
    run_3.Fz_rr=zeros(1,length(s));    run_3.Fz_rr    = calllib("libfastestlapc","download_vector_table_variable",run_3.Fz_rr, length(s), 'run_3/Fz_rr');
    run_3.dissipation_fl = calllib("libfastestlapc","download_vector_table_variable",zeros(1,length(s)), length(s), 'run_3/front_axle.left_tire.dissipation');
    run_3.dissipation_fr = calllib("libfastestlapc","download_vector_table_variable",zeros(1,length(s)), length(s), 'run_3/front_axle.right_tire.dissipation');
    run_3.dissipation_rl = calllib("libfastestlapc","download_vector_table_variable",zeros(1,length(s)), length(s), 'run_3/rear_axle.left_tire.dissipation');
    run_3.dissipation_rr = calllib("libfastestlapc","download_vector_table_variable",zeros(1,length(s)), length(s), 'run_3/rear_axle.right_tire.dissipation');
    engine_energy_3 = calllib("libfastestlapc","download_scalar_table_variable",'run_3/integral_quantities.engine-energy');
    run_3.energy_fl = zeros(1,length(s));
    run_3.energy_fr = zeros(1,length(s));
    run_3.energy_rl = zeros(1,length(s));
    run_3.energy_rr = zeros(1,length(s));
    
    for i = 2 : length(s)
        run_3.energy_fl(i) = run_3.energy_fl(i-1) + 0.5*(run_3.time(i)-run_3.time(i-1))*(run_3.dissipation_fl(i-1) + run_3.dissipation_fl(i));
        run_3.energy_fr(i) = run_3.energy_fr(i-1) + 0.5*(run_3.time(i)-run_3.time(i-1))*(run_3.dissipation_fr(i-1) + run_3.dissipation_fr(i));
        run_3.energy_rl(i) = run_3.energy_rl(i-1) + 0.5*(run_3.time(i)-run_3.time(i-1))*(run_3.dissipation_rl(i-1) + run_3.dissipation_rl(i));
        run_3.energy_rr(i) = run_3.energy_rr(i-1) + 0.5*(run_3.time(i)-run_3.time(i-1))*(run_3.dissipation_rr(i-1) + run_3.dissipation_rr(i));
    end
    
    run_3.q = [run_3.kappa_fl; run_3.kappa_fr;  run_3.kappa_rl; run_3.kappa_rr;...
             run_3.u; run_3.v; run_3.omega; run_3.time; run_3.n; run_3.alpha];
    run_3.qa = [run_3.Fz_fl; run_3.Fz_fr; run_3.Fz_rl; run_3.Fz_rr];
    run_3.control_variables  = [run_3.delta; run_3.throttle; run_3.brake_bias];
    run_3.energy = [run_3.energy_fl; run_3.energy_fr; run_3.energy_rl; run_3.energy_rr];
    run_3.vehicle = vehicle;
    run_3.engine_power = 595.39174549309666;
    calllib("libfastestlapc","clear_tables_by_prefix",'run_3/');    
    
    restart = false;
end

close all;
figure_folder = 'Figures_2';

blue   = [0   0.447000000000000   0.741];
orange = [0.850000000000000   0.325000000000000   0.098000000000000];
background_color =  [13/255, 17/255, 23/255];
text_color = [201,209,217]/255;
screen_size = get(0,'ScreenSize');

% Figure 1: Lift-and-coast vs. Maximum attack
h1 = figure;
h1.Color = background_color;
ax1_position = [0.1,0.45,0.8,0.15];
ax1 = axes('Position',ax1_position);
plot(s,max(100*run_1.throttle,0.0),'LineWidth',2,'Color',blue);
hold on
plot(s,max(100*run_2.throttle,0.0),'LineWidth',2,'Color',orange);
grid on
box on
xlim([s(1),s(end)]);

ax2_position = [0.1,0.25,0.8,0.15];
ax2 = axes('Position',ax2_position);
plot(s,max(-100*run_1.throttle,0.0),'LineWidth',2,'Color',blue);
hold on
plot(s,max(-100*run_2.throttle,0.0),'LineWidth',2,'Color',orange);
grid on
box on
xlim([s(1),s(end)]);

ax3_position = [0.1,0.05,0.8,0.15];
ax3 = axes('Position',ax3_position);
smooth_pos = @(x)(0.5*(x+sqrt(x.*x+0.01)));
throttle_1 = smooth_pos(run_1.throttle);    
engine_energy_1 = cumtrapz(run_1.time, throttle_1*run_1.engine_power);

throttle_2 = smooth_pos(run_2.throttle);    
engine_energy_2 = cumtrapz(run_2.time, throttle_2*run_2.engine_power);

plot(s,(engine_energy_1-engine_energy_2)/engine_energy_1(end)*100.0,'LineWidth',2,'Color',[1,1,1]);
grid on
box on
xlim([s(1),s(end)]);

for ax = [ax1,ax2,ax3]
    ax.Color = background_color;
    ax.GridColor = text_color; ax.XColor = text_color; ax.YColor = text_color;
    
    ax.LineWidth = 1.5;
    
    ax.XMinorTick = 'on';
    ax.YMinorTick = 'on';
    
    XTick = ax.XTick;
    if ax ~= ax3
        ax.XTickLabel = {};
    else
        for i = 1 : numel(ax.XTickLabel)
            ax.XTickLabel{i} = [ax.XTickLabel{i},'m'];
        end
    end
    ax.XTick = XTick;
    
    YTick = ax.YTick;
    for i = 1 : numel(ax.YTickLabel)
        ax.YTickLabel{i} = [ax.YTickLabel{i},'%'];
    end
    ax.YTick = YTick;
    
    ax.FontSize = 14;
end

% Plot circuit
ax4_position = [0.1,0.68,0.8,0.22];
ax4 = axes('Position',ax4_position);

r_center_rot = zeros(2,numel(s));

theta = -deg2rad(58);
for i = 1 : numel(s)
   r_center_rot(:,i) = [cos(theta),sin(theta);-sin(theta),cos(theta)]*[track_data.x_center(i);track_data.y_center(i)]; 
end

ax4.Color = background_color;
ax4.GridColor = text_color; ax4.XColor = text_color; ax4.YColor = text_color;
plot(r_center_rot(1,:),-r_center_rot(2,:),'-w','LineWidth',2);
hold on
track_f = @(s_i)(interp1(s,r_center_rot',s_i));
s_values = 0:500:4500;

for s_i = s_values
    p_i = track_f(s_i);
    text(p_i(1),-p_i(2),num2str(s_i),'FontName','Formula1','FontSize',14,'Color',text_color,'EdgeColor',text_color,'BackgroundColor',background_color,'HorizontalAlignment','center');
end


axis equal;
ax4.Visible = 'off';

annotation('textbox',[ax1_position(1),ax1_position(2)+1.05*ax1_position(4),ax1_position(3),0.03],'string','Throttle [%]','FontSize',15,...
    'FontName','Formula1','interpreter','none','HorizontalAlignment','center','VerticalAlignment','middle',...
    'LineStyle','none','FontWeight','bold','Margin',0,'FitBoxtoText','on','Color',text_color);


annotation('textbox',[ax2_position(1),ax2_position(2)+1.05*ax2_position(4),ax2_position(3),0.03],'string','Brake [%]','FontSize',15,...
    'FontName','Formula1','interpreter','none','HorizontalAlignment','center','VerticalAlignment','middle',...
    'LineStyle','none','FontWeight','bold','Margin',0,'FitBoxtoText','on','Color',text_color);

annotation('textbox',[ax3_position(1),ax3_position(2)+1.05*ax3_position(4),ax3_position(3),0.03],'string','Saved energy [%]','FontSize',15,...
    'FontName','Formula1','interpreter','none','HorizontalAlignment','center','VerticalAlignment','middle',...
    'LineStyle','none','FontWeight','bold','Margin',0,'FitBoxtoText','on','Color',text_color);

annotation('textbox',[0.29,0.9,1.0,0.1],'string','Maximum attack','FontSize',20,...
    'FontName','Formula1','interpreter','none','VerticalAlignment','middle',...
    'LineStyle','none','FontWeight','bold','Margin',0,'FitBoxtoText','on','Color',blue);
annotation('textbox',[0.5,0.9,1.0,0.1],'string','vs.','FontSize',20,...
    'FontName','Formula1','interpreter','none','VerticalAlignment','middle',...
    'LineStyle','none','FontWeight','bold','Margin',0,'FitBoxtoText','on','Color',text_color);
annotation('textbox',[0.54,0.9,1.0,0.1],'string',['Lift-and-coast (+',num2str(run_2.time(end)-run_1.time(end),'%.3f'),'s)'],'FontSize',20,...
    'FontName','Formula1','interpreter','none','VerticalAlignment','middle',...
    'LineStyle','none','FontWeight','bold','Margin',0,'FitBoxtoText','on','Color',orange);
set(h1, 'InvertHardcopy', 'off');

h1.Position = [0,0,0.5*screen_size(3),0.35*screen_size(3)];
print([figure_folder,'Figure1'],'-dpng','-r300');
savefig(h1,[figure_folder,'Figure1']);


% Figure 1: Lift-and-coast vs. Engine turned down
h2 = figure;
h2.Color = background_color;
ax1_position = [0.1,0.45,0.8,0.15];
ax1 = axes('Position',ax1_position);
plot(s,max(100*run_3.throttle,0.0),'LineWidth',2,'Color',blue);
hold on
plot(s,max(100*run_2.throttle,0.0),'LineWidth',2,'Color',orange);
grid on
box on
xlim([s(1),s(end)]);

ax2_position = [0.1,0.25,0.8,0.15];
ax2 = axes('Position',ax2_position);
hold on
smooth_pos = @(x)(0.5*(x+sqrt(x.*x+0.01)));
throttle_3 = smooth_pos(run_3.throttle);    
engine_energy_3 = cumtrapz(run_3.time, throttle_3*run_3.engine_power);

plot(s,(engine_energy_1-engine_energy_3)/engine_energy_1(end)*100.0,'LineWidth',2,'Color',blue);
plot(s,(engine_energy_1-engine_energy_2)/engine_energy_1(end)*100.0,'LineWidth',2,'Color',orange);
grid on
box on
xlim([s(1),s(end)]);

ax3_position = [0.1,0.05,0.8,0.15];
ax3 = axes('Position',ax3_position);
hold on
plot(s,run_3.time-run_1.time,'LineWidth',2,'Color',blue);
plot(s,run_2.time-run_1.time,'LineWidth',2,'Color',orange);
grid on
box on
xlim([s(1),s(end)]);

for ax = [ax1,ax2,ax3]
    ax.Color = background_color;
    ax.GridColor = text_color; ax.XColor = text_color; ax.YColor = text_color;
    
    ax.LineWidth = 1.5;
    
    ax.XMinorTick = 'on';
    ax.YMinorTick = 'on';
    
    XTick = ax.XTick;
    if ax ~= ax3
        ax.XTickLabel = {};
    else
        for i = 1 : numel(ax.XTickLabel)
            ax.XTickLabel{i} = [ax.XTickLabel{i},'m'];
        end
    end
    ax.XTick = XTick;
    
    YTick = ax.YTick;
    if ax ~= ax3
        for i = 1 : numel(ax.YTickLabel)
            ax.YTickLabel{i} = [ax.YTickLabel{i},'%'];
        end
    else
        for i = 1 : numel(ax.YTickLabel)
            ax.YTickLabel{i} = ['+',num2str(str2num(ax.YTickLabel{i}),'%.3f'),'s'];
        end
    end
    ax.YTick = YTick;
    
    ax.FontSize = 14;
end

% Plot circuit
ax4_position = [0.1,0.68,0.8,0.22];
ax4 = axes('Position',ax4_position);

r_center_rot = zeros(2,numel(s));

theta = -deg2rad(58);
for i = 1 : numel(s)
   r_center_rot(:,i) = [cos(theta),sin(theta);-sin(theta),cos(theta)]*[track_data.x_center(i);track_data.y_center(i)]; 
end

ax4.Color = background_color;
ax4.GridColor = text_color; ax4.XColor = text_color; ax4.YColor = text_color;
plot(r_center_rot(1,:),-r_center_rot(2,:),'-w','LineWidth',2);
hold on
track_f = @(s_i)(interp1(s,r_center_rot',s_i));
s_values = 0:500:4500;

for s_i = s_values
    p_i = track_f(s_i);
    text(p_i(1),-p_i(2),num2str(s_i),'FontName','Formula1','FontSize',14,'Color',text_color,'EdgeColor',text_color,'BackgroundColor',background_color);
end


axis equal;
ax4.Visible = 'off';

annotation('textbox',[ax1_position(1),ax1_position(2)+1.05*ax1_position(4),ax1_position(3),0.03],'string','Throttle [%]','FontSize',15,...
    'FontName','Formula1','interpreter','none','HorizontalAlignment','center','VerticalAlignment','middle',...
    'LineStyle','none','FontWeight','bold','Margin',0,'FitBoxtoText','on','Color',text_color);


annotation('textbox',[ax2_position(1),ax2_position(2)+1.05*ax2_position(4),ax2_position(3),0.03],'string','Saved energy [%]','FontSize',15,...
    'FontName','Formula1','interpreter','none','HorizontalAlignment','center','VerticalAlignment','middle',...
    'LineStyle','none','FontWeight','bold','Margin',0,'FitBoxtoText','on','Color',text_color);

annotation('textbox',[ax3_position(1),ax3_position(2)+1.05*ax3_position(4),ax3_position(3),0.03],'string','Delta time [s]','FontSize',15,...
    'FontName','Formula1','interpreter','none','HorizontalAlignment','center','VerticalAlignment','middle',...
    'LineStyle','none','FontWeight','bold','Margin',0,'FitBoxtoText','on','Color',text_color);

annotation('textbox',[0.20,0.9,1.0,0.1],'string',['Turn down engine (+',num2str(run_3.time(end)-run_1.time(end),'%.3f'),'s)'],'FontSize',20,...
    'FontName','Formula1','interpreter','none','VerticalAlignment','middle',...
    'LineStyle','none','FontWeight','bold','Margin',0,'FitBoxtoText','on','Color',blue);
annotation('textbox',[0.545,0.9,1.0,0.1],'string','vs.','FontSize',20,...
    'FontName','Formula1','interpreter','none','VerticalAlignment','middle',...
    'LineStyle','none','FontWeight','bold','Margin',0,'FitBoxtoText','on','Color',text_color);
annotation('textbox',[0.59,0.9,1.0,0.1],'string',['Lift-and-coast (+',num2str(run_2.time(end)-run_1.time(end),'%.3f'),'s)'],'FontSize',20,...
    'FontName','Formula1','interpreter','none','VerticalAlignment','middle',...
    'LineStyle','none','FontWeight','bold','Margin',0,'FitBoxtoText','on','Color',orange);
set(h2, 'InvertHardcopy', 'off');

h2.Position = [0,0,0.5*screen_size(3),0.35*screen_size(3)];
print([figure_folder,'Figure2'],'-dpng','-r300');
savefig(h2,[figure_folder,'Figure2']);

h3 = represent_turn(s,{run_1,run_2},track_data,87,133,"Turn 1",{blue,orange},-deg2rad(58),background_color,text_color);
set(h3, 'InvertHardcopy', 'off');
print([figure_folder,'Figure3'],'-dpng','-r300');
savefig(h3,[figure_folder,'Figure3']);r


h4 = represent_turn(s,{run_1,run_2},track_data,433,500,"Turn 10",{blue,orange},-deg2rad(58),background_color,text_color);
set(h4, 'InvertHardcopy', 'off');
print([figure_folder,'Figure4'],'-dpng','-r300');
savefig(h3,[figure_folder,'Figure4']);r
