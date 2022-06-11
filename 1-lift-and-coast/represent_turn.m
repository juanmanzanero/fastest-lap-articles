function h = represent_turn(s,run,track,i_start,i_end,title,color,theta_circuit,background_color,text_color)

h = figure;
screen_size = get(0,'ScreenSize');
h.Position = [0 0 screen_size(4) screen_size(4)];
h.Color = background_color;
% (1) Title
annotation('textbox',[0.0,0.95,1.0,0.03],'string',title,'FontSize',20,...
    'FontName','Formula1','interpreter','none','HorizontalAlignment','center','VerticalAlignment','middle',...
    'LineStyle','none','FontWeight','bold','Margin',0,'FitBoxtoText','on','Color',text_color);

% (2) GPS plot
ax_gps = represent_gps(s,run,track,i_start,i_end,color,background_color,text_color);

% (3) Speed plot
ax_speed = represent_speed(s,run,track,i_start,i_end,color,background_color,text_color);

% (4) Represent accelerations
represent_acceleration(s,run,track,i_start,i_end,color, background_color, text_color);

% (5) Represent throttle and brake
represent_throttle_and_brake(s,run,track,i_start,i_end,color, background_color, text_color);

% (6) Represent steering
represent_steering(s,run,track,i_start,i_end,color, background_color, text_color);

% (7) Represent full circuit
represent_full_circuit(s,run{1},track,i_start,i_end,color{1},theta_circuit, background_color, text_color);

% (8) Represent FL logo
ax_logo = axes('Position',[0.01,0.93,0.06,0.06]);
imagesc(imread('logo.jpg'));

% (9) Represent engine energy
represent_engine_energy(s,run,track,i_start,i_end,color, background_color, text_color);

% (10) Represent tires energies
represent_tire_energy(s,run,track,i_start,i_end,color, background_color, text_color);

% (11) Represent time delta
represent_time_delta(s,run,track,i_start,i_end,color, background_color, text_color);

ax_logo.Visible = 'off';
end

function ax = represent_gps(s,run,track,i_start,i_end,color,background_color,text_color)
dt = 0.5;
time_start = run{1}.time(i_start);
time_end   = run{1}.time(i_end);

ax = axes('Position', compute_position(6,1,5,1));
ax.Color = background_color;
plot(run{1}.x(i_start:i_end),-run{1}.y(i_start:i_end))

axis equal
hold on
ax.FontSize = 1.0e-8;
set(gca,'xtick',[])
set(gca,'ytick',[])
x_lim = ax.XLim + 0.05*diff(ax.XLim)*[-1,1];
y_lim = ax.YLim + 0.05*diff(ax.YLim)*[-1,1];

% [13/255, 17/255, 23/255]
patch([track.x_left, track.x_right], [-track.y_left, -track.y_right], text_color);

for i = 1 : numel(run)
    run_i = run{i};
    for t_i = 0:dt:time_end-time_start
        
        s_i = interp1(run_i.q(8,:), s, run_i.time(i_start) + t_i,'spline')';
        q_i = interp1(run_i.q(8,:), run_i.q', run_i.time(i_start) + t_i,'spline')';
        qa_i = interp1(run_i.q(8,:), run_i.qa', run_i.time(i_start) + t_i,'spline')';
        u_i = interp1(run_i.q(8,:), run_i.control_variables', run_i.time(i_start) + t_i,'spline')';
        
        
        x_rr = calllib("libfastestlapc","get_vehicle_property",run_i.vehicle, q_i, qa_i, u_i, s_i, 'rear_axle.right_tire.x');
        y_rr = calllib("libfastestlapc","get_vehicle_property",run_i.vehicle, q_i, qa_i, u_i, s_i, 'rear_axle.right_tire.y');
        psi = calllib("libfastestlapc","get_vehicle_property",run_i.vehicle, q_i, qa_i, u_i, s_i, 'psi');
        plot_f1(x_rr, -y_rr, rad2deg(psi)+180, 3.6, 1.52,'generic',color{i}/max(color{i}));
        
    end
end
ax.XLim = x_lim;
ax.YLim = y_lim;

%plot([x_lim(1),x_lim(1),x_lim(2),x_lim(2),x_lim(1)], [y_lim(1),y_lim(2),y_lim(2),y_lim(1),y_lim(1)],'-k','LineWidth',1.5);

ax.Visible = 'off';
end

function ax = represent_speed(s,run,track,i_start,i_end, color, background_color, text_color)

position = compute_position(7,2,1,1);
ax_position = position;
ax = axes('Position', ax_position);
ax.Color = background_color;
ax.GridColor = text_color; ax.XColor = text_color; ax.YColor = text_color;
hold on
dv_axis = 50;
for i = 1 : numel(run)
    run_i = run{i};
    plot(s(i_start:i_end)-s(i_start),3.6*run_i.u(i_start:i_end),'LineWidth',2,'Color',color{i});
end

grid on
box on
ymax = ceil(max(3.6*run{1}.u(i_start:i_end))/dv_axis)*dv_axis;
ymin = floor(min(3.6*run{1}.u(i_start:i_end))/dv_axis)*dv_axis;

for i = 2 : numel(run)
    ymax = max(ymax,ceil(max(3.6*run{i}.u(i_start:i_end))/25)*25);
    ymin = min(ymin,floor(min(3.6*run{i}.u(i_start:i_end))/25)*25);
end

xlim([0,s(i_end)-s(i_start)]);
ylim([ymin,ymax]);
ax.LineWidth = 1.5;
ax.YTick = 0:dv_axis:400;

ax.XTickLabel={};

ax.XMinorTick = 'on';
ax.YMinorTick = 'on';

annotation('textbox',[ax_position(1),ax_position(2)+0.9*ax_position(4),ax_position(3),0.03],'string','Speed [km/h]','FontSize',10,...
    'FontName','Formula1','interpreter','none','HorizontalAlignment','center','VerticalAlignment','middle',...
    'LineStyle','none','FontWeight','bold','Margin',0,'FitBoxtoText','on','Color',text_color);
end

function ax = represent_steering(s,run,track,i_start,i_end, color, background_color, text_color)

position = compute_position(8,2,1,1);
ax_position = position;
ax = axes('Position', ax_position);
ax.Color = background_color;
ax.GridColor = text_color; ax.XColor = text_color; ax.YColor = text_color;
hold on;
for i = 1 : numel(run)
    run_i = run{i};
    plot(s(i_start:i_end)-s(i_start),rad2deg(run_i.delta(i_start:i_end)),'LineWidth',2,'Color',color{i});
end
grid on
box on
xlim([0,s(i_end)-s(i_start)]);
ax.LineWidth = 1.5;

ax.XTickLabel = {};
ax.XMinorTick = 'on';
ax.YMinorTick = 'on';

annotation('textbox',[ax_position(1),ax_position(2)+0.9*ax_position(4),ax_position(3),0.03],'string','Steering [deg]','FontSize',10,...
    'FontName','Formula1','interpreter','none','HorizontalAlignment','center','VerticalAlignment','middle',...
    'LineStyle','none','FontWeight','bold','Margin',0,'FitBoxtoText','on','Color',text_color);
end

function ax = represent_engine_energy(s,run,track,i_start,i_end, color, background_color, text_color)

position = compute_position(5,1,1,1);
ax_position = position;
ax = axes('Position', ax_position);
ax.Color = background_color;
ax.GridColor = text_color; ax.XColor = text_color; ax.YColor = text_color;
hold on;
for i = 1 : numel(run)
    throttle = max(0,run{i}.throttle(i_start:i_end));
    
    engine_energy = cumtrapz(run{i}.time(i_start:i_end), throttle*run{i}.engine_power);
    
    plot(s(i_start:i_end)-s(i_start),1.0e-3*engine_energy,'LineWidth',2,'Color',color{i});
end
grid on
box on
xlim([0,s(i_end)-s(i_start)]);
ax.LineWidth = 1.5;

ax.XTickLabel = {};
ax.XMinorTick = 'on';
ax.YMinorTick = 'on';

annotation('textbox',[ax_position(1),ax_position(2)+0.9*ax_position(4),ax_position(3),0.03],'string','Engine energy [MJ]','FontSize',10,...
    'FontName','Formula1','interpreter','none','HorizontalAlignment','center','VerticalAlignment','middle',...
    'LineStyle','none','FontWeight','bold','Margin',0,'FitBoxtoText','on','Color',text_color);
end

function ax = represent_tire_energy(s,run,track,i_start,i_end, color, background_color, text_color)


maximum_energy = -1.0e-6*min([run{1}.energy_fl(i_end)-run{1}.energy_fl(i_start),run{1}.energy_fr(i_end)-run{1}.energy_fr(i_start),...
    run{1}.energy_rl(i_end)-run{1}.energy_rl(i_start),run{1}.energy_rr(i_end)-run{1}.energy_rr(i_start)]);

for i = 2 : numel(run)
    maximum_energy = max(maximum_energy,-1.0e-6*min([run{i}.energy_fl(i_end)-run{i}.energy_fl(i_start),run{i}.energy_fr(i_end)-run{i}.energy_fr(i_start),...
    run{i}.energy_rl(i_end)-run{i}.energy_rl(i_start),run{i}.energy_rr(i_end)-run{i}.energy_rr(i_start)]));
end
% Front left
ax_position = compute_position(4,1,1,1);
ax = axes('Position', ax_position);
ax.Color = background_color;
ax.GridColor = text_color; ax.XColor = text_color; ax.YColor = text_color;
hold on
for i = 1 : numel(run)
    plot(s(i_start:i_end)-s(i_start),1.0e-6*(run{i}.energy_fl(i_start)-run{i}.energy_fl(i_start:i_end)),'LineWidth',2,'Color',color{i});
end
grid on
box on
xlim([0,s(i_end)-s(i_start)]);
ax.LineWidth = 1.5;
ax.XTickLabel={};

ax.XMinorTick = 'on';
ax.YMinorTick = 'on';
ylim([0,maximum_energy]);
annotation('textbox',[ax_position(1),ax_position(2)+0.9*ax_position(4),ax_position(3),0.03],'string','Front left tire energy [MJ]','FontSize',10,...
    'FontName','Formula1','interpreter','none','HorizontalAlignment','center','VerticalAlignment','middle',...
    'LineStyle','none','FontWeight','bold','Margin',0,'FitBoxtoText','on','Color',text_color);


% Front right
ax_position = compute_position(4,2,1,1);
ax = axes('Position', ax_position);
ax.Color = background_color;
ax.GridColor = text_color; ax.XColor = text_color; ax.YColor = text_color;
hold on
for i = 1 : numel(run)
    plot(s(i_start:i_end)-s(i_start),1.0e-6*(run{i}.energy_fr(i_start)-run{i}.energy_fr(i_start:i_end)),'LineWidth',2,'Color',color{i});
end
grid on
box on
xlim([0,s(i_end)-s(i_start)]);
ax.LineWidth = 1.5;
ax.XTickLabel={};

ax.XMinorTick = 'on';
ax.YMinorTick = 'on';
ylim([0,maximum_energy]);
annotation('textbox',[ax_position(1),ax_position(2)+0.9*ax_position(4),ax_position(3),0.03],'string','Front right tire energy [MJ]','FontSize',10,...
    'FontName','Formula1','interpreter','none','HorizontalAlignment','center','VerticalAlignment','middle',...
    'LineStyle','none','FontWeight','bold','Margin',0,'FitBoxtoText','on','Color',text_color);

% Rear left
ax_position = compute_position(3,1,1,1);
ax = axes('Position', ax_position);
ax.Color = background_color;
ax.GridColor = text_color; ax.XColor = text_color; ax.YColor = text_color;
hold on
for i = 1 : numel(run)
    plot(s(i_start:i_end)-s(i_start),1.0e-6*(run{i}.energy_rl(i_start)-run{i}.energy_rl(i_start:i_end)),'LineWidth',2,'Color',color{i});
end
grid on
box on
xlim([0,s(i_end)-s(i_start)]);
ax.LineWidth = 1.5;

for i = 1 : numel(ax.XTickLabel)
    ax.XTickLabel{i} = [ax.XTickLabel{i},'m'];
end

ax.XMinorTick = 'on';
ax.YMinorTick = 'on';
ylim([0,maximum_energy]);
annotation('textbox',[ax_position(1),ax_position(2)+0.9*ax_position(4),ax_position(3),0.03],'string','Rear left tire energy [MJ]','FontSize',10,...
    'FontName','Formula1','interpreter','none','HorizontalAlignment','center','VerticalAlignment','middle',...
    'LineStyle','none','FontWeight','bold','Margin',0,'FitBoxtoText','on','Color',text_color);

% Rear right
ax_position = compute_position(3,2,1,1);
ax = axes('Position', ax_position);
ax.Color = background_color;
ax.GridColor = text_color; ax.XColor = text_color; ax.YColor = text_color;
hold on
for i = 1 : numel(run)
    plot(s(i_start:i_end)-s(i_start),1.0e-6*(run{i}.energy_rr(i_start)-run{i}.energy_rr(i_start:i_end)),'LineWidth',2,'Color',color{i});
end
grid on
box on
xlim([0,s(i_end)-s(i_start)]);
ax.LineWidth = 1.5;
for i = 1 : numel(ax.XTickLabel)
    ax.XTickLabel{i} = [ax.XTickLabel{i},'m'];
end

ax.XMinorTick = 'on';
ax.YMinorTick = 'on';
ylim([0,maximum_energy]);
annotation('textbox',[ax_position(1),ax_position(2)+0.9*ax_position(4),ax_position(3),0.03],'string','Rear right tire energy [MJ]','FontSize',10,...
    'FontName','Formula1','interpreter','none','HorizontalAlignment','center','VerticalAlignment','middle',...
    'LineStyle','none','FontWeight','bold','Margin',0,'FitBoxtoText','on','Color',text_color);
end

function ax = represent_throttle_and_brake_contour(s,run,track,i_start,i_end, color, background_color, text_color)

ax_position = compute_position(6,2,1,1);

ax = axes('Position', ax_position);
ax.Color = background_color;
ax.GridColor = text_color; ax.XColor = text_color; ax.YColor = text_color;
hold on;
height = 0.25;
for i = 1 : numel(run)
x_surf = [-2,-1,s];
y_surf = [1-i*height;1-(i-1)*height];
col = [-1,1,run{i}.throttle;-1,1,run{i}.throttle];
cMap = interp1([0;0.5;1],[0 1 0; 1 1 0; 1 0 0],linspace(0,1,256));
surface(x_surf,y_surf,zeros(2,numel(s)+2),-col, 'facecol','interp',...
    'edgecol','no',...
    'linew',4);
plot([s(i_start),s(i_start),s(i_end),s(i_end),s(i_start)],[y_surf(1),y_surf(2),y_surf(2),y_surf(1),y_surf(1)],'-k','LineWidth',1.5);

end
ylim([0,1]);
xlim([s(i_start),s(i_end)]);
ax.Visible = 'off';
colormap(cMap)
annotation('textbox',[ax_position(1),ax_position(2)+0.9*ax_position(4),ax_position(3),0.03],'string','Throttle and brake','FontSize',10,...
    'FontName','Formula1','interpreter','none','HorizontalAlignment','center','VerticalAlignment','middle',...
    'LineStyle','none','FontWeight','bold','Margin',0,'FitBoxtoText','on','Color',text_color);

ax_arrows = axes('Position', [ax_position(1)-0.02,ax_position(2),0.02,ax_position(4)]);
hold on
for i = 1 : numel(run)
%patch([1,1-sqrt(2)*0.5*0.25,1-sqrt(2)*0.5*0.25],[1-0.5*0.25,1.0,0.75],color);
patch([1,1-sqrt(2)*0.5*height,1-sqrt(2)*0.5*height],[1-(i-1)*height-0.5*height,1-(i-1)*height,1-(i)*height],color{i});
end
axis equal
ylim([0,1]);
ax_arrows.Visible = 'off';

end

function ax = represent_throttle_and_brake(s,run,track,i_start,i_end, color, background_color, text_color)

ax_position = compute_position(6,2,1,1);

ax = axes('Position', ax_position);
ax.Color = background_color;
ax.GridColor = text_color; ax.XColor = text_color; ax.YColor = text_color;
hold on;
for i = 1 : numel(run)    
    plot(s(i_start:i_end)-s(i_start),100*min(1.0,max(-1.0,run{i}.throttle(i_start:i_end))),'Color',color{i},'LineWidth',2);
end

ylim([-100,100]);
xlim([0,s(i_end)-s(i_start)]);

grid on
box on

ax.LineWidth = 1.5;

for i = 1:numel(ax.YTickLabel)
    ax.YTickLabel{i} = [ax.YTickLabel{i},'%'];
end

ax.XTickLabel = {};
ax.XMinorTick = 'on';
ax.YMinorTick = 'on';
annotation('textbox',[ax_position(1),ax_position(2)+0.9*ax_position(4),ax_position(3),0.03],'string','Throttle and brake','FontSize',10,...
    'FontName','Formula1','interpreter','none','HorizontalAlignment','center','VerticalAlignment','middle',...
    'LineStyle','none','FontWeight','bold','Margin',0,'FitBoxtoText','on','Color',text_color);


end

function ax = represent_acceleration(s,run,track,i_start,i_end, color, background_color, text_color)

ax_position = compute_position(9,2,2,1);
ax_position(1) = ax_position(1) + 0.52*ax_position(3);
ax_position(3) = 0.48*ax_position(3);

ax = axes('Position', ax_position);
ax.Color = background_color;
ax.GridColor = text_color; ax.XColor = text_color; ax.YColor = text_color;
hold on;
max_ax = 0.0;
max_ay = 0.0;
for i = 1 : numel(run)
    run_i = run{i};
    ax_data = zeros(1,i_end-i_start);
    ay_data = zeros(1,i_end-i_start);
    for i_point = i_start:i_end
        ax_data(i_point-i_start+1) = calllib("libfastestlapc","get_vehicle_property",run_i.vehicle,run_i.q(:,i_point),run_i.qa(:,i_point),run_i.control_variables(:,i_point),s(i_point),'ax')/9.81;
        ay_data(i_point-i_start+1) = calllib("libfastestlapc","get_vehicle_property",run_i.vehicle,run_i.q(:,i_point),run_i.qa(:,i_point),run_i.control_variables(:,i_point),s(i_point),'ay')/9.81;
    end
    
    plot(ay_data,ax_data,'LineWidth',2,'Color',color{i});
    
    max_ax = max(max_ax,max(abs(ax_data)));
    max_ay = max(max_ay,max(abs(ay_data)));
end

max_ax = ceil(max_ax);
max_ay = ceil(max_ay);

max_a = max(max_ax,max_ay);

grid on;
box on;
ax.LineWidth = 1.5;
axis equal
xlim([-max_a,max_a]);
ylim([-max_a,max_a]);
ax.XMinorTick = 'on';
ax.YMinorTick = 'on';

annotation('textbox',[ax_position(1),ax_position(2)+ax_position(4),ax_position(3),0.03],'string','Longitudinal vs. Lateral acceleration [g]','FontSize',10,...
    'FontName','Formula1','interpreter','none','HorizontalAlignment','center','VerticalAlignment','middle',...
    'LineStyle','none','FontWeight','bold','Margin',0,'FitBoxtoText','on','Color',text_color);

end

function ax = represent_full_circuit(s,run,track,i_start,i_end, color, theta, background_color, text_color)

ax_position = compute_position(9,2,2,1);
ax_position(3) = 0.48*ax_position(3);

r_center_rot = zeros(2,numel(s));

for i = 1 : numel(s)
   r_center_rot(:,i) = [cos(theta),sin(theta);-sin(theta),cos(theta)]*[track.x_center(i);track.y_center(i)]; 
end

ax = axes('Position', ax_position);
ax.Color = background_color;
ax.GridColor = text_color; ax.XColor = text_color; ax.YColor = text_color;
plot(r_center_rot(1,:),-r_center_rot(2,:),'-w','LineWidth',2);

i_start_track = dsearchn([track.x_center;track.y_center]',[run.x(i_start),run.y(i_start)]);
i_end_track = dsearchn([track.x_center;track.y_center]',[run.x(i_end),run.y(i_end)]);

hold on;
grid on;
axis equal;
plot(r_center_rot(1,i_start_track:i_end_track),-r_center_rot(2,i_start_track:i_end_track),'-w','LineWidth',6);
plot(r_center_rot(1,i_start_track:i_end_track),-r_center_rot(2,i_start_track:i_end_track),'LineWidth',4,'Color','#FFFF00');

ax.Visible = 'off';
end

function ax = represent_time_delta(s,run,track,i_start,i_end, color, background_color, text_color)

position = compute_position(5,2,1,1);
ax_position = position;
ax = axes('Position', ax_position);
ax.Color = background_color;
ax.GridColor = text_color; ax.XColor = text_color; ax.YColor = text_color;
hold on;
plot(s(i_start:i_end)-s(i_start),zeros(1,i_end-i_start+1),'LineWidth',2,'Color',color{1});

for i = 2 : numel(run)
   plot(s(i_start:i_end)-s(i_start), run{i}.time(i_start:i_end)-run{i}.time(i_start) - (run{1}.time(i_start:i_end)-run{1}.time(i_start)),'Color',color{i},'LineWidth',2);
end

grid on
xlim([0,s(i_end)-s(i_start)]);
ax.LineWidth = 1.5;

for i = 1 : numel(ax.YTickLabel)
    if ( str2num(ax.YTickLabel{i}) >= 0)
        ax.YTickLabel{i} = ['+',num2str(str2num(ax.YTickLabel{i}),'%.3f'),'s'];
    else
        ax.YTickLabel{i} = [num2str(str2num(ax.YTickLabel{i}),'%.3f'),'s'];
    end
end
box on;
ax.XTickLabel={};

ax.XMinorTick = 'on';
ax.YMinorTick = 'on';

annotation('textbox',[ax_position(1),ax_position(2)+0.9*ax_position(4),ax_position(3),0.03],'string','Time delta [s]','FontSize',10,...
    'FontName','Formula1','interpreter','none','HorizontalAlignment','center','VerticalAlignment','middle',...
    'LineStyle','none','FontWeight','bold','Margin',0,'FitBoxtoText','on','Color',text_color);
end

function pos = compute_position(i_start, j_start, i_span, j_span, color)

top_margin = 0.075;
left_margin = 0.05;
right_margin = 0.05;
bottom_margin = 0.05;
horizontal_margin = 0.035;
vertical_margin = 0.05;

n_rows = 10;
n_cols = 2;


assert(i_start > 0);
assert(j_start > 0);
assert(i_span > 0);
assert(j_span > 0);
assert(i_span <= n_rows);
assert(j_span <= n_cols);

box_width = (1-left_margin-right_margin-(n_cols-1)*vertical_margin)/n_cols;
box_height = (1-top_margin-bottom_margin-(n_rows-1)*horizontal_margin)/n_rows;

% (i_start,j_start) give the location of the origin
origin = [left_margin + (box_width+vertical_margin)*(j_start-1), bottom_margin + (horizontal_margin+box_height)*(i_start-1)];

width = box_width + (j_span-1)*(box_width+vertical_margin);
height = box_height + (i_span-1)*(box_height+horizontal_margin);

pos = [origin(1), origin(2), width, height];


end