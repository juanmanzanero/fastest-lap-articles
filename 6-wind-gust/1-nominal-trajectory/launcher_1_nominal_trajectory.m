global fastest_lap

fastest_lap = 'flap';
track_file = 'cota_uniform_1000.xml';
vehicle_file = 'neutral-car.xml';
skip_run_full_optimal_laptime = true;

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

% (3) Load car
vehicle_name = 'car';
calllib(fastest_lap, 'create_vehicle_from_xml', vehicle_name, vehicle_file);

% (4) Run optimal laptime for the full circuit

% (4.1) Set options
options =           '<options>';
options = [options, '    <output_variables>'];
options = [options, '        <prefix>run/</prefix>'];
options = [options, '    </output_variables>'];
options = [options, '    <control_variables>'];
options = [options, '        <chassis.brake-bias optimal_control_type="hypermesh">'];
options = [options, '            <hypermesh>0.0, 990.0, 1526.0, 1925.0, 2589.0, 3024.0, 3554.0</hypermesh>'];
options = [options, '        </chassis.brake-bias>'];
options = [options, '    </control_variables>'];
options = [options, '</options>'];


if skip_run_full_optimal_laptime
    % (4.2) Skip computation
    load('full_lap');
else
    % (4.2) Run
    calllib(fastest_lap, 'optimal_laptime', vehicle_name, circuit_name, n_points, s, options);

    % (4.3) Get results
    simulation_data = get_run_data('run/', n_points);
end

% (5) Run optimal laptime only for the zone of interest
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
calllib(fastest_lap,'optimal_laptime', vehicle_name, circuit_name, numel(corner_study.s), corner_study.s, options);

% (5.3) Download data
corner_study.simulation_data = get_run_data('run_corner/', numel(corner_study.s));

% (6.0) Compute the corner by using propagation
run_propagation.q = repmat(corner_study.q_start(:),1,numel(corner_study.s));
run_propagation.qa = repmat(corner_study.qa_start(:),1,numel(corner_study.s));
run_propagation.u = [corner_study.simulation_data.("front-axle.steering-angle")(:)'; ...
    corner_study.simulation_data.("rear-axle.boost")(:)'; ...
    corner_study.simulation_data.("chassis.throttle")(:)'; ...
    corner_study.simulation_data.("chassis.brake-bias")(:)'];

options =          '<options>';
options = [options,    '<sigma> 0.5 </sigma>'];
options = [options,    '<error_tolerance> 1.0e-10 </error_tolerance>'];
options = [options,'</options>'];
for i = 1 : numel(corner_study.s)-1
    [run_propagation.q(:,i+1), run_propagation.qa(:,i+1)] = calllib(fastest_lap,'propagate_vehicle',run_propagation.q(:,i),run_propagation.qa(:,i), run_propagation.u(:,i), vehicle_name, circuit_name, corner_study.s(i), corner_study.s(i+1)-corner_study.s(i), run_propagation.u(:,i+1), true, options);
end

save('air_in_calm_simulation');

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

