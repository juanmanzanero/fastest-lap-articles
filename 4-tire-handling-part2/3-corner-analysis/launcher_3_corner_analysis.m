clear all
%clc
close all

fastest_lap = 'flap';
fastest_lap_path = '/Users/juanmanzanero/Documents/software/fastest-lap';
fastest_lap_version = '0.4';
fastest_lap_suffix = 'dylib';
vehicle_database = 'ferrari-2022-australia.xml';

kmh = 1/3.6;
n_max_char = 100;

dtor = onCleanup(@()(clean(fastest_lap)));

loadlibrary([fastest_lap_path,'/build/lib/libfastestlapc.',fastest_lap_version,'.',fastest_lap_suffix],...
    [fastest_lap_path,'/src/main/c/fastestlapc.h'],'alias',fastest_lap);

% (1) Get information from the car
[n_state, n_algebraic, n_control, n_outputs] = calllib(fastest_lap,'vehicle_type_get_sizes',0,0,0,0,'f1-3dof');
[key_name, state_names, algebraic_state_names, control_names, output_names] = calllib(fastest_lap,'vehicle_type_get_names',blanks(n_max_char),repmat({blanks(n_max_char)},1,n_state),repmat({blanks(n_max_char)},1,n_algebraic),repmat({blanks(n_max_char)},1,n_control),repmat({blanks(n_max_char)},1,n_outputs),n_max_char,'f1-3dof');

% (2) Load vehicle
vehicle = 'car';
calllib(fastest_lap,'create_vehicle_from_xml',vehicle,vehicle_database);
calllib(fastest_lap,'vehicle_set_parameter',vehicle,'vehicle/chassis/roll_balance_coefficient',0.3);
% (3) Compute gg diagram
velocity = 100*kmh;
n_points_gg = 100;
[ay, ax_max, ax_min] = calllib(fastest_lap,'gg_diagram', zeros(1,n_points_gg), zeros(1,n_points_gg), zeros(1,n_points_gg), vehicle, velocity, n_points_gg);

figure;
plot(ay,ax_max)
% (4) Get the approximation of the ay acceleration for ax = 0
positive_accelerations = find(ax_max > 0);
ax_max_f = @(ay_val)(interp1(ay(positive_accelerations(end):end), ax_max(positive_accelerations(end):end), ay_val));
ay_for_no_acceleration = fsolve(ax_max_f,ay(end));

% (5) Solve a steady-state for this acceleration
[ss.q,ss.qa,ss.u] = calllib(fastest_lap,'steady_state', zeros(1,n_state), zeros(1,n_algebraic), zeros(1,n_control), vehicle, velocity, ax_max(end), ay(end));

% (5.1) Get extra variables
track = 'straight';
calllib(fastest_lap,'create_track_from_xml',track,'straight.xml');
calllib(fastest_lap,'vehicle_change_track',vehicle,track);

ss.lambda_fl = calllib(fastest_lap,'vehicle_get_output',vehicle,ss.q,ss.qa,ss.u,0,'front_axle.left_tire.lambda');
ss.lambda_fr = calllib(fastest_lap,'vehicle_get_output',vehicle,ss.q,ss.qa,ss.u,0,'front_axle.right_tire.lambda');
ss.lambda_rl = calllib(fastest_lap,'vehicle_get_output',vehicle,ss.q,ss.qa,ss.u,0,'rear_axle.left_tire.lambda');
ss.lambda_rr = calllib(fastest_lap,'vehicle_get_output',vehicle,ss.q,ss.qa,ss.u,0,'rear_axle.right_tire.lambda');


% (6) Plot stuff
car_data = readstruct(vehicle_database);
lambda=linspace(0,deg2rad(12),100); 

figure;
subplot(2,2,1);
[Fx,Fy,rho] = pacejka_model(-ss.qa(1)*660*9.81,ss.q(1),lambda,car_data.front_tire);
[Fx_fl,Fy_fl,rho_fl] = pacejka_model(-ss.qa(1)*660*9.81,ss.q(1),ss.lambda_fl,car_data.front_tire);
plot(rho,Fy,rho_fl,Fy_fl,'o');
title(sprintf('rho: %g',rho_fl))
ylim([-1e4,1e4])

subplot(2,2,2);
[Fx,Fy,rho] = pacejka_model(-ss.qa(2)*660*9.81,ss.q(2),lambda,car_data.front_tire);
[Fx_fr,Fy_fr,rho_fr] = pacejka_model(-ss.qa(2)*660*9.81,ss.q(2),ss.lambda_fr,car_data.front_tire);
plot(rho,Fy,rho_fr,Fy_fr,'o');
title(sprintf('rho: %g',rho_fr))
ylim([-1e4,1e4])

subplot(2,2,3);
[Fx,Fy,rho] = pacejka_model(-ss.qa(3)*660*9.81,ss.q(3),lambda,car_data.rear_tire);
[Fx_rl,Fy_rl,rho_rl] = pacejka_model(-ss.qa(3)*660*9.81,ss.q(3),ss.lambda_rl,car_data.rear_tire);
plot(rho,Fy,rho_rl,Fy_rl,'o');
title(sprintf('rho: %g',rho_rl))
ylim([-1e4,1e4])

subplot(2,2,4);
[Fx,Fy,rho] = pacejka_model(-ss.qa(4)*660*9.81,ss.q(4),lambda,car_data.rear_tire);
[Fx_rr,Fy_rr,rho_rr] = pacejka_model(-ss.qa(4)*660*9.81,ss.q(4),ss.lambda_rr,car_data.rear_tire);
plot(rho,Fy,rho_rr,Fy_rr,'o');
title(sprintf('rho: %g',rho_rr))
ylim([-1e4,1e4])

sgtitle(sprintf('delta = %gdeg',rad2deg(ss.u(1))));

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
    Fx = mu_x_max.*Fz.*sin(Qx.*rho).*kappa./(kappa_max.*rho);
    Fy = mu_y_max.*Fz.*sin(Qy.*rho).*lambda./(lambda_max.*rho);
end


function clean(fastest_lap)

if libisloaded(fastest_lap)
    unloadlibrary(fastest_lap)
end

end