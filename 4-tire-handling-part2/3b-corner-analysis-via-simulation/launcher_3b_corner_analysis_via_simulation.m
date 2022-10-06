clear all
%clc
close all

fastest_lap = 'flap';
fastest_lap_path = 'C:\Users\jmanz\Documents\software\fastest-lap-vs\fastest-lap';
fastest_lap_library_path = 'C:\Users\jmanz\Documents\software\fastest-lap-vs\vs\x64\Release';
fastest_lap_version = '-0.4';
fastest_lap_suffix = 'dll';
vehicle_database = 'neutral-car.xml';

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

% (3) Create a circular track
n_points = 50;
create_circular_track(0.015,n_points,12);

% (3.1) Load it
track = 'track';
calllib(fastest_lap,'create_track_from_xml',track,'circular_track.xml');
arclength = calllib(fastest_lap,'track_download_data',zeros(1,n_points), track, n_points, 'arclength');
% (4) Run an optimal laptime
options = '<options> <output_variables> <prefix> run/ </prefix> </output_variables> </options>';
calllib(fastest_lap,'optimal_laptime',vehicle,track,n_points,arclength,options);

% (4.1) Get data: the right side is the loaded one
run.time = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run/time');
run.u = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run/chassis.velocity.x');
run.v = calllib(fastest_lap,'download_vector',zeros(1,n_points), n_points, 'run/chassis.velocity.y');
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

function create_circular_track(curvature,n_points,width)

kappa = curvature*ones(1,n_points+1);
theta = linspace(0,2*pi,n_points+1);

x = cos(theta)/curvature;
y = sin(theta)/curvature;


nl = 0.5*width*ones(1,n_points+1);
nr = nl;

x_left = x + nl.*cos(theta);
y_left = y + nl.*sin(theta);

x_right = x - nr.*cos(theta);
y_right = y - nr.*sin(theta);

dx = diff(x);
dy = diff(y);

darclength = sqrt(dx.*dx + dy.*dy);

arclength = [0,cumsum(darclength)];

track_length = arclength(end);

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
node.appendChild(docNode.createTextNode(num2str(theta(1:end-1))));
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

function clean(fastest_lap)

if libisloaded(fastest_lap)
    unloadlibrary(fastest_lap)
end

end