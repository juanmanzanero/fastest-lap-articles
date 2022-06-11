if (~exist('restart','var'))
    restart = true;
end

set(groot,'defaultAxesFontName','Formula1');

if restart
    clear all
    clc
    close all
    restart = false;
    
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
    options = [options,    '    </variables>'];
    options = [options,    '</save_variables>'];
    options = [options,'</options>'];
    
    % (4.2) Compute
    calllib("libfastestlapc","optimal_laptime",vehicle,track,length(s),s,options);
    
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
    options = [options,            '<upper_bound> 28.0 </upper_bound>'];
    options = [options,        '</engine-energy>'];
    options = [options,    '</integral_constraints>'];
    options = [options,'</options>'];
    
    % (5.2) Compute
    calllib("libfastestlapc","optimal_laptime",vehicle,track,length(s),s,options);
    
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
end

close all;

blue   = [0   0.447000000000000   0.741];
orange = [0.850000000000000   0.325000000000000   0.098000000000000];
background_color =  [13/255, 17/255, 23/255];
text_color = [201,209,217]/255;

h = represent_turn(s,{run_1,run_2},track_data,433,500,"Turn 10",{blue,orange},-deg2rad(58),background_color,text_color);
set(h, 'InvertHardcopy', 'off');
print('test.png','-dpng','-r300');
